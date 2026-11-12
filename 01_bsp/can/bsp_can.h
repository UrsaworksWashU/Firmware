/**
 * @file    bsp_can.h
 * @author  eason
 * @brief   CAN board support: start the buses, send frames, dispatch received frames.
 *
 *          bsp_can knows nothing about motors or any other device protocol.
 *          - Receive: a module registers (bus, id, callback, owner). When a frame with that
 *            bus + id arrives, bsp calls the callback from the CAN RX interrupt.
 *          - Transmit: a module hands over (bus, id, data, len) and bsp sends it.
 *          No HAL types appear in this header, so another chip (e.g. FDCAN on the DM-MC02)
 *          only needs a new bsp_can.c, not changes in the modules above.
 *
 * @version beta
 * @date    2026-11-11
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

/* ------------------------------ parameters ------------------------------ */

#define CAN_RX_REGISTER_MAX 16      /* max number of (bus, id) pairs that can be registered */
#define CAN_DATA_LEN_MAX    8       /* classic CAN carries at most 8 data bytes */
#define CAN_STD_ID_MAX      0x7FFu  /* standard identifiers are 11 bits */

/* ------------------------------ types ------------------------------ */

/** Which CAN bus. Callers use this instead of CAN_HandleTypeDef. */
typedef enum
{
    CAN_BUS_1 = 0,
    CAN_BUS_2,
    CAN_BUS_NUM, /* number of buses; use for array sizes and range checks */
} CAN_Bus_e;

/** Result of a bsp_can call. CAN_OK is 0, so `if (status)` means "something failed". */
typedef enum
{
    CAN_OK = 0,
    CAN_ERR_PARAM,     /* bad bus, id, length, or NULL pointer */
    CAN_ERR_FULL,      /* RegisterRx: no free slot left (raise CAN_RX_REGISTER_MAX) */
    CAN_ERR_DUPLICATE, /* RegisterRx: this bus + id is already registered */
    CAN_ERR_BUSY,      /* Transmit: all 3 TX mailboxes are occupied */
    CAN_ERR_HAL,       /* a HAL call returned an error */
} CAN_Status_e;

/**
 * Receive callback, called from the CAN RX interrupt.
 * Keep it short: decode and store only. Never call HAL_Delay or wait on HAL_GetTick here.
 *
 * @param bus   bus the frame arrived on
 * @param id    standard identifier of the frame
 * @param data  received bytes; valid only during this call, copy what you need
 * @param len   number of bytes in data (0..8)
 * @param owner the pointer given at registration, e.g. the motor object this frame belongs to
 */
typedef void (*CAN_RxCallback_t)(CAN_Bus_e bus, uint32_t id,
                                 const uint8_t *data, uint8_t len, void *owner);

/** What a module hands to bsp when it wants to receive one identifier. */
typedef struct
{
    CAN_Bus_e bus;             /* which bus to listen on */
    uint32_t id;               /* standard identifier to receive (0..0x7FF) */
    CAN_RxCallback_t callback; /* called when a matching frame arrives */
    void *owner;               /* passed back to callback unchanged; may be NULL */
} CAN_RxConfig_t;

/** Per-bus counters. Watch them in Ozone to see whether the bus is healthy. */
typedef struct
{
    uint32_t tx_ok;        /* frames accepted into a TX mailbox */
    uint32_t tx_fail;      /* Transmit calls that failed (mailboxes full or HAL error) */
    uint32_t rx_count;     /* frames received */
    uint32_t rx_unclaimed; /* received frames with no registered callback */
} CAN_Stats_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Configure filters, start both buses and enable RX interrupts.
 * Call once after MX_CAN1_Init() / MX_CAN2_Init(), before any RegisterRx or Transmit.
 */
CAN_Status_e BSP_CAN_Init(void);

/**
 * Register a callback for one (bus, id) pair.
 * Call during initialization, not from an interrupt.
 * The config is copied, so it may live on the caller's stack.
 */
CAN_Status_e BSP_CAN_RegisterRx(const CAN_RxConfig_t *config);

/**
 * Queue one standard data frame for transmission. Does not wait:
 * returns CAN_ERR_BUSY at once if all TX mailboxes are full.
 * Safe to call from task or interrupt context.
 *
 * @param data bytes to send; copied before returning
 * @param len  0..8
 */
CAN_Status_e BSP_CAN_Transmit(CAN_Bus_e bus, uint32_t id, const uint8_t *data, uint8_t len);

/** Read-only access to a bus's counters, or NULL for an invalid bus. */
const CAN_Stats_t *BSP_CAN_GetStats(CAN_Bus_e bus);

#endif /* BSP_CAN_H */