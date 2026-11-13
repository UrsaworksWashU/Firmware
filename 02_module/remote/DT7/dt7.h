/**
 * @file    dt7.h
 * @author  eason
 * @brief   DJI DT7 transmitter + DR16 receiver (DBUS protocol).
 *
 *          DBUS frame: 18 bytes every 14 ms, little-endian bit stream:
 *            ch0..ch3   11 bits each, 364..1684, center 1024   (sticks)
 *            s1, s2     2 bits each, 1 = up, 3 = middle, 2 = down (switches)
 *            mouse x/y/z int16, mouse left/right uint8, keys uint16
 *            bytes 16-17: ch4 (left dial) on recent DR16 firmware, same range as the sticks
 *
 *          Robustness (stricter than HNU):
 *          - a frame whose length is not 18 bytes is dropped (partial frame after power-up)
 *          - a frame with any channel or switch out of range is dropped as a whole
 *            (the last good data is kept), instead of zeroing single channels
 *          - no new good frame for DT7_OFFLINE_TIMEOUT_MS -> offline, GetData returns safe zeros
 *
 *          Single instance: a robot has one DBUS receiver, so the state lives inside dt7.c.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef DT7_H
#define DT7_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_uart.h"

/* ------------------------------ constants ------------------------------ */

#define DT7_FRAME_LEN          18u
#define DT7_CH_OFFSET          1024
#define DT7_CH_MAX_ABS         660   /* sticks and dial: -660 .. +660 after removing the offset */
#define DT7_OFFLINE_TIMEOUT_MS 100u  /* about 7 lost frames */

/* Keyboard bits in DT7_Data_t.keys (from the PC client through the link) */
#define DT7_KEY_W     (1u << 0)
#define DT7_KEY_S     (1u << 1)
#define DT7_KEY_D     (1u << 2)
#define DT7_KEY_A     (1u << 3)
#define DT7_KEY_SHIFT (1u << 4)
#define DT7_KEY_CTRL  (1u << 5)
#define DT7_KEY_Q     (1u << 6)
#define DT7_KEY_E     (1u << 7)
#define DT7_KEY_R     (1u << 8)
#define DT7_KEY_F     (1u << 9)
#define DT7_KEY_G     (1u << 10)
#define DT7_KEY_Z     (1u << 11)
#define DT7_KEY_X     (1u << 12)
#define DT7_KEY_C     (1u << 13)
#define DT7_KEY_V     (1u << 14)
#define DT7_KEY_B     (1u << 15)

/* ------------------------------ types ------------------------------ */

/* Raw protocol values. NONE is never sent by the receiver; it marks "offline". */
typedef enum
{
    DT7_SW_NONE = 0,
    DT7_SW_UP = 1,
    DT7_SW_DOWN = 2,
    DT7_SW_MID = 3,
} DT7_Switch_e;

/* Sticks: right = +x, up = +y. TODO verify on hardware: which switch is s1 and which is s2.
 * HNU decodes bits 4-5 of byte 5 as the RIGHT switch; DJI's demo comment says left. */
typedef struct
{
    int16_t right_x;   /* ch0 */
    int16_t right_y;   /* ch1 */
    int16_t left_x;    /* ch2 */
    int16_t left_y;    /* ch3 */
    int16_t dial;      /* ch4, the wheel on the top left */
    DT7_Switch_e switch_right;
    DT7_Switch_e switch_left;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;  /* 1 = pressed */
    uint8_t mouse_right;
    uint16_t keys;       /* DT7_KEY_* bits */
} DT7_Data_t;

typedef struct
{
    uint32_t frames_ok;
    uint32_t frames_bad_len;   /* not 18 bytes: partial frame, normal right after power-up */
    uint32_t frames_bad_range; /* a channel or switch out of range: corrupt frame */
} DT7_Stats_t;

/* ------------------------------ functions ------------------------------ */

/** Register with bsp_uart on the given port and start receiving. */
UART_Status_e DT7_Init(UART_Port_e port);

/**
 * Copy the latest good frame (interrupt-safe snapshot).
 * @return true if online. When offline, *out is filled with zeros and DT7_SW_NONE switches,
 *         so a caller that ignores the return value still sees "sticks centered, nothing pressed".
 */
bool DT7_GetData(DT7_Data_t *out);

/** Counters, for watching in Ozone. */
const DT7_Stats_t *DT7_GetStats(void);

#endif /* DT7_H */