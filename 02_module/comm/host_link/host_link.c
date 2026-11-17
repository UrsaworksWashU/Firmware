/**
 * @file    host_link.c
 * @author  eason
 * @brief   'U' id len payload crc16 'N' frames over USB. See host_link.h for the frame layout.
 *
 *          Parser idea: collect bytes into frame_buf, starting at a 'U'. After every byte, check
 *          whether frame_buf already holds a complete frame. If the candidate turns out to be
 *          garbage (len too big, bad crc, no 'N'), it may still contain the start of a real frame
 *          (a false 'U' inside float data is common), so drop only the first byte and rescan the
 *          rest for the next 'U' instead of throwing everything away.
 *
 * @version beta
 * @date    2026-11-13
 * @copyright Copyright (c) 2026
 *
 */

#include "host_link.h"
#include "bsp_critical.h"
#include "bsp_usb.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* ------------------------------ frame layout ------------------------------ */

#define IDX_SOF     0u
#define IDX_ID      1u
#define IDX_LEN     2u
#define IDX_PAYLOAD 3u
#define HEADER_LEN  3u /* 'U' id len */
#define TRAILER_LEN 3u /* crc16 'N' */
#define MAX_FRAME   (HEADER_LEN + HOST_LINK_MAX_PAYLOAD + TRAILER_LEN)

/* ------------------------------ private state ------------------------------ */

/* Handlers: entries [0, handler_count) are valid. */
static HostLink_RxConfig_t handlers[HOST_LINK_MAX_HANDLERS];
static volatile uint8_t handler_count;

/* Parser: frame_buf[0, frame_count) holds the current candidate frame; frame_buf[0] is 'U'. */
static uint8_t frame_buf[MAX_FRAME];
static uint16_t frame_count;

static bool initialized;
static HostLink_Stats_t link_stats;

/* ------------------------------ crc ------------------------------ */

/**
 * CRC-16/MCRF4XX, bit by bit. Gives the same result as the 256-entry table in the vision code
 * (the table is just these 8 steps precomputed for every possible byte).
 * Reflected: bytes enter from the low bit, so the register shifts right and the polynomial
 * 0x1021 appears bit-reversed as 0x8408.
 */
static uint16_t Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/* ------------------------------ parser ------------------------------ */

/** The candidate is garbage: drop its first byte and restart at the next 'U' already buffered. */
static void Resync(void)
{
    uint16_t next = 1u;
    while (next < frame_count && frame_buf[next] != HOST_LINK_SOF)
    {
        next++;
    }
    frame_count = (uint16_t)(frame_count - next);
    memmove(frame_buf, &frame_buf[next], frame_count);
}

/** A frame passed crc and 'N': hand the payload to whoever registered its id. */
static void Dispatch(uint8_t id, const uint8_t *payload, uint8_t len)
{
    const uint8_t count = handler_count;
    for (uint8_t i = 0; i < count; i++)
    {
        if (handlers[i].id != id)
        {
            continue;
        }
        if (handlers[i].len != len)
        {
            link_stats.bad_len++;
            return;
        }
        link_stats.frames_ok++;
        handlers[i].handler(payload, len, handlers[i].owner);
        return;
    }
    link_stats.unknown_id++;
}

/**
 * Check the candidate in frame_buf. Loops because a Resync can leave bytes that already form
 * (the start of) the next candidate.
 */
static void TryParse(void)
{
    while (frame_count >= HEADER_LEN)
    {
        const uint8_t len = frame_buf[IDX_LEN];
        if (len > HOST_LINK_MAX_PAYLOAD)
        {
            link_stats.bad_len++;
            Resync();
            continue;
        }

        const uint16_t frame_len = (uint16_t)(HEADER_LEN + len + TRAILER_LEN);
        if (frame_count < frame_len)
        {
            return; /* wait for more bytes */
        }

        /* frame_count == frame_len here: we check after every byte, so it never runs past. */
        const uint16_t crc_pos = (uint16_t)(HEADER_LEN + len);
        const uint16_t crc_rx = (uint16_t)(frame_buf[crc_pos] | ((uint16_t)frame_buf[crc_pos + 1u] << 8));
        if (Crc16(frame_buf, crc_pos) != crc_rx)
        {
            link_stats.crc_err++;
            Resync();
            continue;
        }
        if (frame_buf[frame_len - 1u] != HOST_LINK_EOF)
        {
            link_stats.eof_err++;
            Resync();
            continue;
        }

        Dispatch(frame_buf[IDX_ID], &frame_buf[IDX_PAYLOAD], len);
        frame_count = 0u;
        return;
    }
}

void HostLink_Feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL)
    {
        return;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        const uint8_t byte = data[i];
        if (frame_count == 0u && byte != HOST_LINK_SOF)
        {
            continue; /* between frames: skip everything that is not a 'U' (e.g. 'SP' packets) */
        }
        frame_buf[frame_count] = byte;
        frame_count++;
        TryParse();
    }
}

/* ------------------------------ transport ------------------------------ */

/** bsp_usb rx callback (USB interrupt). */
static void UsbRx(const uint8_t *data, uint16_t len, void *owner)
{
    (void)owner;
    HostLink_Feed(data, len);
}

/* ------------------------------ public functions ------------------------------ */

HostLink_Status_e HostLink_Init(void)
{
    if (initialized)
    {
        return HOST_LINK_ERR_DUPLICATE;
    }
    const USB_RxConfig_t usb_config = {
        .callback = UsbRx,
        .owner = NULL,
    };
    if (BSP_USB_RegisterRx(&usb_config) != USB_OK)
    {
        return HOST_LINK_ERR_TRANSPORT;
    }
    initialized = true;
    return HOST_LINK_OK;
}

HostLink_Status_e HostLink_RegisterRx(const HostLink_RxConfig_t *config)
{
    if (config == NULL || config->handler == NULL || config->len > HOST_LINK_MAX_PAYLOAD)
    {
        return HOST_LINK_ERR_PARAM;
    }
    const uint8_t count = handler_count;
    for (uint8_t i = 0; i < count; i++)
    {
        if (handlers[i].id == config->id)
        {
            return HOST_LINK_ERR_DUPLICATE;
        }
    }
    if (count >= HOST_LINK_MAX_HANDLERS)
    {
        return HOST_LINK_ERR_FULL;
    }

    /* Same pattern as BSP_USB_RegisterRx: the interrupt may already be parsing, so write the
     * entry completely, then (after the barrier) make it visible by bumping the count. */
    handlers[count] = *config;
    BSP_MemoryBarrier();
    handler_count = (uint8_t)(count + 1u);
    return HOST_LINK_OK;
}

const HostLink_Stats_t *HostLink_GetStats(void)
{
    return &link_stats;
}