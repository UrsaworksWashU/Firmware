/**
 * @file    nav_cmd.c
 * @author  eason
 * @brief   Navigation command decoder. See nav_cmd.h for the payload layout.
 *
 * @version beta
 * @date    2026-11-13
 * @copyright Copyright (c) 2026
 *
 */

#include "nav_cmd.h"
#include "bsp_critical.h"
#include "bsp_time.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* ------------------------------ private state ------------------------------ */

/* Written by the USB interrupt, read by NavCmd_GetData inside a critical section. */
static NavCmd_Data_t latest;
static volatile uint32_t last_frame_ms;
static volatile bool has_frame;

static NavCmd_Stats_t nav_stats;

/* ------------------------------ private helpers ------------------------------ */

/**
 * Little-endian float32 from a byte pointer that may be unaligned (payload starts at frame byte 3).
 * memcpy, not *(float *)p: an FPU load (VLDR) from an unaligned address is a HardFault on the M4.
 * Both the host and the STM32 are little-endian, so the bytes copy straight across.
 */
static float ReadFloatLE(const uint8_t *p)
{
    float value;
    memcpy(&value, p, sizeof(value));
    return value;
}

/** NaN-safe: a NaN fails every comparison, so !(|v| <= max) is true for NaN, inf and too fast. */
static bool SpeedValid(float v)
{
    return fabsf(v) <= NAV_CMD_MAX_ABS_MPS;
}

/** host_link handler (USB interrupt). len is guaranteed to be NAV_CMD_PAYLOAD_LEN. */
static void OnPacket(const uint8_t *payload, uint8_t len, void *owner)
{
    (void)len;
    (void)owner;

    NavCmd_Data_t decoded;
    decoded.vx_mps = ReadFloatLE(&payload[0]);
    decoded.vy_mps = ReadFloatLE(&payload[4]);
    decoded.yaw_ref_rad = ReadFloatLE(&payload[8]);
    const uint8_t flags = payload[12];
    decoded.spin = (flags & NAV_CMD_FLAG_SPIN) != 0u;
    decoded.yaw_ref_valid = (flags & NAV_CMD_FLAG_YAW_REF_VALID) != 0u;

    if (!SpeedValid(decoded.vx_mps) || !SpeedValid(decoded.vy_mps) || !isfinite(decoded.yaw_ref_rad))
    {
        nav_stats.frames_bad_value++;
        return; /* keep the previous command; it times out if no good packet follows */
    }

    /* No critical section here: the only reader runs at thread level and blocks this interrupt. */
    latest = decoded;
    last_frame_ms = BSP_Time_GetMs();
    has_frame = true;
    nav_stats.frames_ok++;
}

/* ------------------------------ public functions ------------------------------ */

HostLink_Status_e NavCmd_Init(void)
{
    const HostLink_RxConfig_t config = {
        .id = HOST_LINK_ID_NAV_CMD,
        .len = NAV_CMD_PAYLOAD_LEN,
        .handler = OnPacket,
        .owner = NULL,
    };
    return HostLink_RegisterRx(&config);
}

bool NavCmd_GetData(NavCmd_Data_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    const uint32_t state = BSP_EnterCritical();
    const NavCmd_Data_t copy = latest;
    const uint32_t frame_ms = last_frame_ms;
    const bool any_frame = has_frame;
    BSP_ExitCritical(state);

    /* "now" after frame_ms, for the same unsigned-wrap reason as DT7_GetData. */
    const uint32_t now = BSP_Time_GetMs();
    const bool online = any_frame && (now - frame_ms) < NAV_CMD_TIMEOUT_MS;

    if (online)
    {
        *out = copy;
    }
    else
    {
        memset(out, 0, sizeof(*out));
    }
    return online;
}

const NavCmd_Stats_t *NavCmd_GetStats(void)
{
    return &nav_stats;
}