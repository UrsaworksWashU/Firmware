/**
 * @file    nav_cmd.h
 * @author  eason
 * @brief   Navigation command from the host PC (host_link packet id 0x01, ~20 Hz while moving).
 *
 *          Payload (13 bytes, little-endian, packed):
 *            [0..3]   vx       float32  m/s, forward along the barrel
 *            [4..7]   vy       float32  m/s, to the left of the barrel
 *            [8..11]  yaw_ref  float32  rad, gimbal yaw when the host computed this command
 *            [12]     flags    uint8    bit0 = spin (small gyro), bit1 = yaw_ref valid
 *
 *          Frame (see host_link.h): 'U' 0x01 0x0D payload crc16 'N', 19 bytes.
 *
 *          The host stops sending when it has no goal, so silence is normal: after
 *          NAV_CMD_TIMEOUT_MS without a good packet the command is offline and reads as zeros.
 *          Packets with NaN / inf or a speed above NAV_CMD_MAX_ABS_MPS are dropped whole.
 *
 * @version beta
 * @date    2026-11-13
 * @copyright Copyright (c) 2026
 *
 */

#ifndef NAV_CMD_H
#define NAV_CMD_H

#include "host_link.h"
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------ constants ------------------------------ */

#define NAV_CMD_PAYLOAD_LEN     13u
#define NAV_CMD_TIMEOUT_MS      300u  /* agreed with the navigation side */
#define NAV_CMD_MAX_ABS_MPS     5.0f  /* sanity limit, far above anything the chassis can do */

#define NAV_CMD_FLAG_SPIN          0x01u
#define NAV_CMD_FLAG_YAW_REF_VALID 0x02u

/* ------------------------------ types ------------------------------ */

typedef struct
{
    float vx_mps;       /* forward along the barrel */
    float vy_mps;       /* left of the barrel */
    float yaw_ref_rad;
    bool spin;
    bool yaw_ref_valid;
} NavCmd_Data_t;

typedef struct
{
    uint32_t frames_ok;
    uint32_t frames_bad_value; /* NaN / inf / speed over the limit */
} NavCmd_Stats_t;

/* ------------------------------ functions ------------------------------ */

/** Register for packet id 0x01. Call after HostLink_Init. */
HostLink_Status_e NavCmd_Init(void);

/**
 * Copy the latest command into *out.
 * @return true if a good packet arrived within NAV_CMD_TIMEOUT_MS. false: offline, *out is all zeros.
 */
bool NavCmd_GetData(NavCmd_Data_t *out);

/** Counters, for watching in Ozone. */
const NavCmd_Stats_t *NavCmd_GetStats(void);

#endif /* NAV_CMD_H */