/**
 * @file    host_link.h
 * @author  eason
 * @brief   Framed packets between the board and the host PC (navigation), over USB CDC.
 *
 *          Frame (all multi-byte fields little-endian, no padding):
 *
 *            +-----+----+-----+-----------------+----------+-----+
 *            | 'U' | id | len | payload[len]    | crc16    | 'N' |
 *            +-----+----+-----+-----------------+----------+-----+
 *              1     1    1     0..MAX_PAYLOAD    2 (LE)     1
 *
 *          crc16 covers 'U' through the last payload byte. It is the same CRC as the auto-aim
 *          'SP' packets (CRC-16/MCRF4XX: poly 0x8408 reflected, init 0xFFFF, no final xor;
 *          check value for "123456789" is 0x6F91), so the vision side reuses its crc code.
 *
 *          The auto-aim 'SP' packets share the same USB link. They are not ours: the parser
 *          skips them byte by byte until it finds a 'U' that starts a valid frame.
 *
 *          Receiving: USB hands over packets of 1..64 bytes that do not line up with frames.
 *          HostLink_Feed runs a byte-by-byte state machine that remembers where it is between
 *          calls, so split frames are reassembled and merged frames are all found.
 *          A complete, valid frame is dispatched to the handler registered for its id.
 *
 * @version beta
 * @date    2026-11-13
 * @copyright Copyright (c) 2026
 *
 */

#ifndef HOST_LINK_H
#define HOST_LINK_H

#include <stdint.h>

/* ------------------------------ constants ------------------------------ */

/* 'U' ... 'N' = UrsaWorks Navigation */
#define HOST_LINK_SOF          0x55u /* 'U' start of frame */
#define HOST_LINK_EOF          0x4Eu /* 'N' end of frame */
#define HOST_LINK_MAX_PAYLOAD  32u   /* longer frames are rejected (bounds the parser buffer) */
#define HOST_LINK_MAX_HANDLERS 4u    /* registered ids */

/* Packet ids. High bit = direction: 0x0_ host -> board, 0x8_ board -> host. */
#define HOST_LINK_ID_NAV_CMD   0x01u /* host -> board: vx, vy, yaw_ref, flags */

/* ------------------------------ types ------------------------------ */

typedef enum
{
    HOST_LINK_OK = 0,
    HOST_LINK_ERR_PARAM,     /* NULL pointer, or len > HOST_LINK_MAX_PAYLOAD */
    HOST_LINK_ERR_DUPLICATE, /* id already has a handler */
    HOST_LINK_ERR_FULL,      /* HOST_LINK_MAX_HANDLERS reached */
    HOST_LINK_ERR_TRANSPORT, /* USB refused the registration */
} HostLink_Status_e;

/**
 * Called from the USB interrupt with one valid frame's payload. len always equals the length
 * given at registration. payload is only valid during the call: decode or copy, then return.
 */
typedef void (*HostLink_Handler_t)(const uint8_t *payload, uint8_t len, void *owner);

typedef struct
{
    uint8_t id;
    uint8_t len; /* expected payload length; frames with another len are counted as bad_len */
    HostLink_Handler_t handler;
    void *owner; /* passed back to handler, a hand-written "this" */
} HostLink_RxConfig_t;

typedef struct
{
    uint32_t frames_ok;  /* valid frames delivered to a handler */
    uint32_t crc_err;    /* crc mismatch */
    uint32_t eof_err;    /* crc ok but the last byte was not 'N' */
    uint32_t bad_len;    /* len too long, or not the registered length for that id */
    uint32_t unknown_id; /* valid frame, but nobody registered its id */
} HostLink_Stats_t;

/* ------------------------------ functions ------------------------------ */

/** Become the USB rx owner. Call once, after MX_USB_DEVICE_Init. */
HostLink_Status_e HostLink_Init(void);

/** Register a handler for one packet id. Call during init, before packets of that id matter. */
HostLink_Status_e HostLink_RegisterRx(const HostLink_RxConfig_t *config);

/**
 * Push received bytes through the parser. The USB callback calls this; a PC test program can
 * call it directly with hand-made byte streams. Must always be called from the same context.
 */
void HostLink_Feed(const uint8_t *data, uint16_t len);

/** Counters, for watching in Ozone. */
const HostLink_Stats_t *HostLink_GetStats(void);

#endif /* HOST_LINK_H */