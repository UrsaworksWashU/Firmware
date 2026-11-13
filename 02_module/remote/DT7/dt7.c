/**
 * @file    dt7.c
 * @author  eason
 * @brief   DJI DT7 / DR16 DBUS decoder. See dt7.h for the protocol and the robustness rules.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "dt7.h"
#include "bsp_critical.h"
#include "bsp_time.h"
#include <stddef.h>
#include <string.h>

#define DT7_CH_RAW_MIN 364
#define DT7_CH_RAW_MAX 1684

/* ------------------------------ private state ------------------------------ */

/* Written by the UART interrupt, read by DT7_GetData inside a critical section. */
static DT7_Data_t latest;              /* last good frame */
static volatile uint32_t last_frame_ms;
static volatile bool has_frame;        /* at least one good frame since boot */

static DT7_Stats_t stats;

/* ------------------------------ private helpers ------------------------------ */

static bool ChannelRawValid(uint16_t raw)
{
    return raw >= DT7_CH_RAW_MIN && raw <= DT7_CH_RAW_MAX;
}

static bool SwitchValid(uint8_t raw)
{
    return raw == DT7_SW_UP || raw == DT7_SW_MID || raw == DT7_SW_DOWN;
}

/** Raw 364..1684 -> -660..660. Only call on a value that passed ChannelRawValid. */
static int16_t ChannelToSigned(uint16_t raw)
{
    return (int16_t)((int32_t)raw - DT7_CH_OFFSET);
}

/**
 * Decode one 18-byte frame into *out.
 * @return false if any field is out of range (out is then partly written and must be discarded).
 *
 * The four stick channels are 11-bit fields packed back to back, least significant bit first:
 *   ch0 = bits  0..10   -> byte0 (8 bits) + low 3 bits of byte1
 *   ch1 = bits 11..21   -> high 5 bits of byte1 + low 6 bits of byte2
 *   ch2 = bits 22..32   -> high 2 bits of byte2 + byte3 + low 1 bit of byte4
 *   ch3 = bits 33..43   -> high 7 bits of byte4 + low 4 bits of byte5
 *   s1  = bits 44..45, s2 = bits 46..47 (high 4 bits of byte5)
 */
static bool DecodeFrame(const uint8_t *b, DT7_Data_t *out)
{
    const uint16_t ch0 = (uint16_t)((b[0] | (b[1] << 8)) & 0x07FF);
    const uint16_t ch1 = (uint16_t)(((b[1] >> 3) | (b[2] << 5)) & 0x07FF);
    const uint16_t ch2 = (uint16_t)(((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07FF);
    const uint16_t ch3 = (uint16_t)(((b[4] >> 1) | (b[5] << 7)) & 0x07FF);
    const uint8_t s1 = (uint8_t)((b[5] >> 4) & 0x03);
    const uint8_t s2 = (uint8_t)((b[5] >> 6) & 0x03);
    const uint16_t ch4 = (uint16_t)((b[16] | (b[17] << 8)) & 0x07FF);

    if (!ChannelRawValid(ch0) || !ChannelRawValid(ch1) ||
        !ChannelRawValid(ch2) || !ChannelRawValid(ch3))
    {
        return false;
    }
    if (!SwitchValid(s1) || !SwitchValid(s2))
    {
        return false;
    }
    /* Older DR16 firmware leaves bytes 16-17 at 0 (no dial). Treat 0 as "no dial", not as an error,
     * otherwise every frame from such a receiver would be rejected. */
    if (ch4 != 0 && !ChannelRawValid(ch4))
    {
        return false;
    }

    out->right_x = ChannelToSigned(ch0);
    out->right_y = ChannelToSigned(ch1);
    out->left_x = ChannelToSigned(ch2);
    out->left_y = ChannelToSigned(ch3);
    out->dial = (ch4 == 0) ? 0 : ChannelToSigned(ch4);

    /* TODO verify on hardware (see dt7.h): s1 = right switch follows HNU. */
    out->switch_right = (DT7_Switch_e)s1;
    out->switch_left = (DT7_Switch_e)s2;

    /* Mouse and keys: plain little-endian integers. */
    out->mouse_x = (int16_t)(uint16_t)(b[6] | (b[7] << 8));
    out->mouse_y = (int16_t)(uint16_t)(b[8] | (b[9] << 8));
    out->mouse_z = (int16_t)(uint16_t)(b[10] | (b[11] << 8));
    out->mouse_left = b[12];
    out->mouse_right = b[13];
    out->keys = (uint16_t)(b[14] | (b[15] << 8));
    return true;
}

/** bsp_uart callback, interrupt context. */
static void OnFrame(UART_Port_e port, const uint8_t *data, uint16_t len, void *owner)
{
    (void)port;  /* single instance: we know which port we registered */
    (void)owner;

    if (len != DT7_FRAME_LEN)
    {
        stats.frames_bad_len++;
        return;
    }

    /* Decode into a local first: a bad frame must not touch the last good data. */
    DT7_Data_t decoded;
    if (!DecodeFrame(data, &decoded))
    {
        stats.frames_bad_range++;
        return;
    }

    /* No critical section needed here: the only reader (DT7_GetData) runs at thread level
     * and blocks this interrupt while it copies. */
    latest = decoded;
    last_frame_ms = BSP_Time_GetMs();
    has_frame = true;
    stats.frames_ok++;
}

/* ------------------------------ public functions ------------------------------ */

UART_Status_e DT7_Init(UART_Port_e port)
{
    const UART_RxConfig_t config = {
        .port = port,
        .callback = OnFrame,
        .owner = NULL, /* single instance, state is file-static */
    };
    return BSP_UART_RegisterRx(&config);
}

bool DT7_GetData(DT7_Data_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    const uint32_t state = BSP_EnterCritical();
    const DT7_Data_t copy = latest;
    const uint32_t frame_ms = last_frame_ms;
    const bool any_frame = has_frame;
    BSP_ExitCritical(state);

    /* Read "now" AFTER frame_ms. If it were read before, a frame arriving in between would make
     * frame_ms newer than now, and the unsigned (now - frame_ms) would wrap to a huge number. */
    const uint32_t now = BSP_Time_GetMs();
    const bool online = any_frame && (now - frame_ms) < DT7_OFFLINE_TIMEOUT_MS;

    if (online)
    {
        *out = copy;
    }
    else
    {
        memset(out, 0, sizeof(*out)); /* sticks centered, switches DT7_SW_NONE, nothing pressed */
    }
    return online;
}

const DT7_Stats_t *DT7_GetStats(void)
{
    return &stats;
}