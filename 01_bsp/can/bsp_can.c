/**
 * @file    bsp_can.c
 * @author  eason
 * @brief   CAN board support for STM32F4 bxCAN (CAN1 + CAN2). See bsp_can.h for the interface.
 *
 *          Design:
 *          - Hardware filters accept every frame into FIFO0; software dispatches by (bus, id).
 *            Bus traffic is small, and software dispatch is easy to watch in the debugger.
 *          - Registrations live in a static table (no malloc), so RAM use is fixed at link time.
 *          - Transmit never waits: if all 3 TX mailboxes are busy it fails right away.
 *
 * @version beta
 * @date    2026-11-11
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_can.h"
#include "can.h" /* hcan1, hcan2 and the HAL CAN API; only this .c file sees HAL types */
#include <string.h>

/* ------------------------------ private state ------------------------------ */

/* CAN_Bus_e -> HAL handle. Index with a CAN_Bus_e value. */
static CAN_HandleTypeDef *const bus_handle[CAN_BUS_NUM] = {
    [CAN_BUS_1] = &hcan1,
    [CAN_BUS_2] = &hcan2,
};

/* Filter banks: the 28 banks are shared. Banks 0..13 belong to CAN1, 14..27 to CAN2. */
#define CAN2_FIRST_FILTER_BANK 14u
static const uint32_t bus_filter_bank[CAN_BUS_NUM] = {
    [CAN_BUS_1] = 0u,
    [CAN_BUS_2] = CAN2_FIRST_FILTER_BANK,
};

/* Registration table. Entries [0, rx_count) are valid.
 * rx_count is volatile because the RX interrupt reads it while RegisterRx may be writing it. */
static CAN_RxConfig_t rx_table[CAN_RX_REGISTER_MAX];
static volatile uint8_t rx_count;

static CAN_Stats_t stats[CAN_BUS_NUM];

/* ------------------------------ private helpers ------------------------------ */

/** Set one filter bank of this bus to "accept every identifier" and route matches to FIFO0. */
static CAN_Status_e ConfigAcceptAllFilter(CAN_Bus_e bus)
{
    CAN_FilterTypeDef filter = {0}; /* zero-init: every field has a defined value */

    filter.FilterBank = bus_filter_bank[bus];
    filter.SlaveStartFilterBank = CAN2_FIRST_FILTER_BANK; /* HAL writes this on every call */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;     /* id   = 0 ...                                  */
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000; /* mask = 0: compare no bits, so every id passes */
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;

    return (HAL_CAN_ConfigFilter(bus_handle[bus], &filter) == HAL_OK) ? CAN_OK : CAN_ERR_HAL;
}

/** HAL handle -> CAN_Bus_e. Returns CAN_BUS_NUM if the handle is not one of ours. */
static CAN_Bus_e BusFromHandle(const CAN_HandleTypeDef *hcan)
{
    for (uint8_t bus = 0; bus < CAN_BUS_NUM; bus++)
    {
        if (bus_handle[bus] == hcan)
        {
            return (CAN_Bus_e)bus;
        }
    }
    return CAN_BUS_NUM;
}

/** Find the registration for (bus, id) and call it. Returns 1 if someone handled the frame. */
static uint8_t Dispatch(CAN_Bus_e bus, uint32_t id, const uint8_t *data, uint8_t len)
{
    const uint8_t count = rx_count; /* read the volatile once */

    for (uint8_t i = 0; i < count; i++)
    {
        const CAN_RxConfig_t *entry = &rx_table[i];
        if (entry->bus == bus && entry->id == id) /* match BOTH bus and id */
        {
            entry->callback(bus, id, data, len, entry->owner);
            return 1;
        }
    }
    return 0;
}

/* ------------------------------ public functions ------------------------------ */

CAN_Status_e BSP_CAN_Init(void)
{
    /* Configure every filter first, then start the buses.
     * Changing filters briefly pauses reception on BOTH buses (they share the filter block),
     * so doing it before anything is started keeps things simple. */
    for (uint8_t bus = 0; bus < CAN_BUS_NUM; bus++)
    {
        if (ConfigAcceptAllFilter((CAN_Bus_e)bus) != CAN_OK)
        {
            return CAN_ERR_HAL;
        }
    }

    for (uint8_t bus = 0; bus < CAN_BUS_NUM; bus++)
    {
        CAN_HandleTypeDef *hcan = bus_handle[bus];

        /* Leave init mode and join the bus. Fails if the RX pin never sees 11 recessive bits. */
        if (HAL_CAN_Start(hcan) != HAL_OK)
        {
            return CAN_ERR_HAL;
        }
        /* Enable the peripheral-side interrupt. The NVIC side is enabled by CubeMX in can.c. */
        if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            return CAN_ERR_HAL;
        }
    }
    return CAN_OK;
}

CAN_Status_e BSP_CAN_RegisterRx(const CAN_RxConfig_t *config)
{
    if (config == NULL || config->callback == NULL ||
        config->bus >= CAN_BUS_NUM || config->id > CAN_STD_ID_MAX)
    {
        return CAN_ERR_PARAM;
    }

    const uint8_t count = rx_count;

    for (uint8_t i = 0; i < count; i++)
    {
        if (rx_table[i].bus == config->bus && rx_table[i].id == config->id)
        {
            return CAN_ERR_DUPLICATE;
        }
    }
    if (count >= CAN_RX_REGISTER_MAX)
    {
        return CAN_ERR_FULL;
    }

    /* Fill the entry completely BEFORE making it visible by bumping rx_count.
     * The RX interrupt only reads entries below rx_count, so it can never see a half-written one.
     * __DMB() stops the compiler and CPU from reordering the two steps. */
    rx_table[count] = *config;
    __DMB();
    rx_count = count + 1;

    return CAN_OK;
}

CAN_Status_e BSP_CAN_Transmit(CAN_Bus_e bus, uint32_t id, const uint8_t *data, uint8_t len)
{
    if (bus >= CAN_BUS_NUM || id > CAN_STD_ID_MAX || len > CAN_DATA_LEN_MAX ||
        (len > 0 && data == NULL))
    {
        return CAN_ERR_PARAM;
    }

    CAN_HandleTypeDef *hcan = bus_handle[bus];

    /* Never wait for a mailbox: waiting here would freeze a control loop or an interrupt. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        stats[bus].tx_fail++;
        return CAN_ERR_BUSY;
    }

    CAN_TxHeaderTypeDef header = {0};
    header.StdId = id;
    header.IDE = CAN_ID_STD;   /* 11-bit standard identifier */
    header.RTR = CAN_RTR_DATA; /* data frame, not a remote request */
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;

    /* HAL_CAN_AddTxMessage always reads 8 bytes, whatever DLC says.
     * Copy into a full 8-byte buffer so a caller passing a shorter array is never over-read. */
    uint8_t buffer[CAN_DATA_LEN_MAX] = {0};
    if (len > 0)
    {
        memcpy(buffer, data, len);
    }

    uint32_t mailbox; /* which mailbox HAL used; not needed afterwards */
    if (HAL_CAN_AddTxMessage(hcan, &header, buffer, &mailbox) != HAL_OK)
    {
        stats[bus].tx_fail++;
        return CAN_ERR_HAL;
    }

    stats[bus].tx_ok++;
    return CAN_OK;
}

const CAN_Stats_t *BSP_CAN_GetStats(CAN_Bus_e bus)
{
    return (bus < CAN_BUS_NUM) ? &stats[bus] : NULL;
}

/* ------------------------------ HAL callback ------------------------------ */

/**
 * The single strong definition of HAL's weak RX FIFO0 callback for the whole program.
 * HAL_CAN_IRQHandler calls it from CAN1_RX0_IRQHandler / CAN2_RX0_IRQHandler.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    const CAN_Bus_e bus = BusFromHandle(hcan);
    if (bus >= CAN_BUS_NUM)
    {
        return;
    }

    CAN_RxHeaderTypeDef header;
    uint8_t data[CAN_DATA_LEN_MAX]; /* HAL always writes all 8 bytes */

    /* Drain the FIFO: several frames may have arrived before this interrupt ran. */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        stats[bus].rx_count++;

        /* Only standard data frames are supported; anything else counts as unclaimed. */
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
        {
            stats[bus].rx_unclaimed++;
            continue; /* skip to the next frame in the FIFO */
        }

        const uint8_t handled = Dispatch(bus, header.StdId, data, (uint8_t)header.DLC);
        if (!handled)
        {
            stats[bus].rx_unclaimed++;
        }
    }
}