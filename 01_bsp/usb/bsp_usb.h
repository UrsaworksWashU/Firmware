/**
 * @file    bsp_usb.h
 * @author  eason
 * @brief   USB virtual COM port (CDC) on board C: raw bytes to/from the host PC (NUC / Jetson).
 *
 *          The PC sees the board as a serial port (/dev/ttyACM0, COMx). Baud rate does not matter:
 *          USB full speed moves data in packets of up to 64 bytes, whatever the PC "baud" says.
 *
 *          Receive: every USB packet the PC sends arrives here as one callback (1..64 bytes).
 *          A packet is NOT a frame: the PC may merge two frames into one packet or split one frame
 *          across two. Finding frame boundaries is the protocol module's job, not bsp's.
 *
 *          Transmit: bsp copies the bytes into its own buffer, so the caller's buffer can be reused
 *          right away. If the previous transmit is still in flight, the new one is refused (BUSY),
 *          never waited for.
 *
 *          There is only one USB port, so there is no port parameter and at most one rx owner.
 *
 * @version beta
 * @date    2026-11-13
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------ constants ------------------------------ */

#define USB_RX_PACKET_MAX 64u  /* full-speed bulk packet size: one callback never exceeds this */
#define USB_TX_BUFFER_SIZE 128u /* longest single transmit */

/* ------------------------------ types ------------------------------ */

typedef enum
{
    USB_OK = 0,
    USB_ERR_PARAM,      /* NULL pointer, len 0 or len > USB_TX_BUFFER_SIZE */
    USB_ERR_DUPLICATE,  /* rx already has an owner */
    USB_ERR_NOT_READY,  /* cable unplugged or the PC has not finished enumeration */
    USB_ERR_BUSY,       /* previous transmit still in flight */
} USB_Status_e;

/**
 * Called from the USB interrupt with one received packet.
 * data is only valid during the call (the next packet reuses the buffer): parse or copy, then return.
 */
typedef void (*USB_RxCallback_t)(const uint8_t *data, uint16_t len, void *owner);

typedef struct
{
    USB_RxCallback_t callback;
    void *owner; /* passed back to callback, a hand-written "this" */
} USB_RxConfig_t;

typedef struct
{
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t tx_ok;
    uint32_t tx_busy;      /* refused: previous transmit not finished */
    uint32_t tx_not_ready; /* refused: not connected */
} USB_Stats_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Pull D+ low for a moment so the PC sees an unplug + replug.
 * Without it, after flashing or a reset (cable still plugged in) the PC never re-enumerates
 * the board and the COM port stays dead. Call in main.c USER CODE SysInit, i.e. BEFORE
 * MX_USB_DEVICE_Init configures PA12 as the USB pin.
 */
void BSP_USB_ForceReenumerate(void);

/** Become the rx owner. Call after MX_USB_DEVICE_Init. */
USB_Status_e BSP_USB_RegisterRx(const USB_RxConfig_t *config);

/** Queue len bytes for the PC. Never blocks. */
USB_Status_e BSP_USB_Transmit(const uint8_t *data, uint16_t len);

/** true once the PC has enumerated the board (cable in, driver loaded). */
bool BSP_USB_IsConnected(void);

/** Counters, for watching in Ozone. */
const USB_Stats_t *BSP_USB_GetStats(void);

/* ------------------------------ glue (not for modules) ------------------------------ */

/**
 * Called ONLY from CDC_Receive_FS in Core's usbd_cdc_if.c (USER CODE 6), before the next packet
 * is armed. Modules use BSP_USB_RegisterRx instead.
 */
void BSP_USB_OnReceive(const uint8_t *data, uint32_t len);

#endif /* BSP_USB_H */
