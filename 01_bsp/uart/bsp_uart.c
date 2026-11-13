/**
 * @file    bsp_uart.c
 * @author  eason
 * @brief   UART receive with DMA + IDLE detection. See bsp_uart.h for the interface.
 *
 *          HAL pieces used:
 *          - HAL_UARTEx_ReceiveToIdle_DMA: start DMA into a buffer and also enable the IDLE interrupt.
 *          - HAL_UARTEx_RxEventCallback: HAL calls it on IDLE (frame ended) or when the buffer is full.
 *            In DMA Normal mode HAL has already stopped reception by then, so we restart it ourselves.
 *          - HAL_UART_ErrorCallback: noise / framing / parity / overrun. With DMA reception HAL
 *            aborts the transfer on these errors, so we restart here too.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_uart.h"
#include "usart.h" /* huart3 and the HAL UART API; only this .c file sees HAL types */
#include <stdbool.h>
#include <stddef.h>

/* ------------------------------ private state ------------------------------ */

/* UART_Port_e -> HAL handle. */
static UART_HandleTypeDef *const port_handle[UART_PORT_NUM] = {
    [UART_PORT_DBUS] = &huart3,
};

/* DMA writes here. Static, so it is in SRAM1 (DMA cannot reach the F4's CCM RAM). */
static uint8_t rx_buffer[UART_PORT_NUM][UART_RX_BUFFER_SIZE];

/* One owner per port. owner_table[port] is valid only when registered[port] is true. */
static UART_RxConfig_t owner_table[UART_PORT_NUM];
static volatile bool registered[UART_PORT_NUM];

static UART_Stats_t stats[UART_PORT_NUM];

/* ------------------------------ private helpers ------------------------------ */

/** HAL handle -> UART_Port_e. Returns UART_PORT_NUM if the handle is not one of ours. */
static UART_Port_e PortFromHandle(const UART_HandleTypeDef *huart)
{
    for (uint8_t port = 0; port < UART_PORT_NUM; port++)
    {
        if (port_handle[port] == huart)
        {
            return (UART_Port_e)port;
        }
    }
    return UART_PORT_NUM;
}

/** (Re)start DMA reception at the beginning of the port's buffer. */
static bool StartReception(UART_Port_e port)
{
    UART_HandleTypeDef *huart = port_handle[port];

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer[port], UART_RX_BUFFER_SIZE) != HAL_OK)
    {
        return false;
    }
    /* HAL also reports the DMA "half transfer" point as an Rx event, which would hand us half a
     * buffer as if it were a frame. We only want IDLE (and buffer full), so turn HT off. */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return true;
}

/* ------------------------------ public functions ------------------------------ */

UART_Status_e BSP_UART_RegisterRx(const UART_RxConfig_t *config)
{
    if (config == NULL || config->callback == NULL || config->port >= UART_PORT_NUM)
    {
        return UART_ERR_PARAM;
    }
    if (registered[config->port])
    {
        return UART_ERR_DUPLICATE;
    }

    /* Same publish order as bsp_can: fill the entry, barrier, then mark it valid.
     * Reception is not running yet, so no interrupt can see the half-filled entry anyway,
     * but keeping one pattern everywhere is easier to reason about. */
    owner_table[config->port] = *config;
    __DMB();
    registered[config->port] = true;

    if (!StartReception(config->port))
    {
        registered[config->port] = false;
        return UART_ERR_HAL;
    }
    return UART_OK;
}

const UART_Stats_t *BSP_UART_GetStats(UART_Port_e port)
{
    return (port < UART_PORT_NUM) ? &stats[port] : NULL;
}

/* ------------------------------ HAL callbacks ------------------------------ */

/**
 * The single strong definition of HAL's weak Rx event callback.
 * Called from USART3_IRQHandler (IDLE) or DMA1_Stream1_IRQHandler (buffer full).
 * Size = number of bytes DMA wrote since reception started.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    const UART_Port_e port = PortFromHandle(huart);
    if (port >= UART_PORT_NUM)
    {
        return;
    }

    if (registered[port])
    {
        const UART_RxConfig_t *entry = &owner_table[port];
        stats[port].rx_frames++;
        entry->callback(port, rx_buffer[port], Size, entry->owner);
    }

    /* Restart only after the owner has returned: DMA writes into the same buffer again. */
    if (!StartReception(port))
    {
        stats[port].restart_fail++;
    }
}

/**
 * The single strong definition of HAL's weak UART error callback.
 * A wrong word length / parity setting shows up here as a steadily rising rx_errors.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const UART_Port_e port = PortFromHandle(huart);
    if (port >= UART_PORT_NUM)
    {
        return;
    }

    stats[port].rx_errors++;
    if (registered[port] && !StartReception(port))
    {
        stats[port].restart_fail++;
    }
}