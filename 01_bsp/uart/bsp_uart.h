/**
 * @file    bsp_uart.h
 * @author  eason
 * @brief   UART receive for board C: DMA + IDLE line detection, one owner per port.
 *
 *          How a frame is received:
 *          DMA copies incoming bytes into a per-port buffer. When the line goes quiet for one
 *          byte time (IDLE), the frame is over: bsp calls the owner's callback with the bytes
 *          received so far, then restarts DMA at the start of the buffer.
 *
 *          The buffer (UART_RX_BUFFER_SIZE) is larger than any frame we expect, so a frame always
 *          ends by IDLE, never by a full buffer, and len is the true frame length.
 *          The module checks len: a wrong length means a partial or corrupt frame.
 *
 *          UART is point-to-point, so unlike CAN each port has at most one owner (no id dispatch).
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/* ------------------------------ constants ------------------------------ */

#define UART_RX_BUFFER_SIZE 64u /* per port; must be larger than the longest frame */

/* ------------------------------ types ------------------------------ */

/* Ports are named by their connector on board C, not by the USART number. */
typedef enum
{
    UART_PORT_DBUS = 0, /* USART3, PC11 RX: DT7/DR16 receiver */
    UART_PORT_NUM,
} UART_Port_e;

typedef enum
{
    UART_OK = 0,
    UART_ERR_PARAM,     /* NULL pointer or port out of range */
    UART_ERR_DUPLICATE, /* the port already has an owner */
    UART_ERR_HAL,       /* HAL refused to start reception */
} UART_Status_e;

/**
 * Called from interrupt context with one received frame.
 * data is only valid during the call (DMA reuses the buffer right after): decode or copy, then return.
 */
typedef void (*UART_RxCallback_t)(UART_Port_e port, const uint8_t *data, uint16_t len, void *owner);

typedef struct
{
    UART_Port_e port;
    UART_RxCallback_t callback;
    void *owner; /* passed back to callback, a hand-written "this" */
} UART_RxConfig_t;

typedef struct
{
    uint32_t rx_frames;   /* IDLE events delivered to the owner */
    uint32_t rx_errors;   /* noise / framing / parity / overrun errors (reception restarted) */
    uint32_t restart_fail;/* HAL refused to restart reception; the port is deaf until the next try */
} UART_Stats_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Become the owner of a port and start receiving.
 * Call after the CubeMX MX_USARTx_UART_Init / MX_DMA_Init functions.
 */
UART_Status_e BSP_UART_RegisterRx(const UART_RxConfig_t *config);

/** Per-port counters, for watching in Ozone. NULL if port is out of range. */
const UART_Stats_t *BSP_UART_GetStats(UART_Port_e port);

#endif /* BSP_UART_H */