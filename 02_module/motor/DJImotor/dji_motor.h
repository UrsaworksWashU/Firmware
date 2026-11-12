/**
 * @file    dji_motor.h
 * @author  eason
 * @brief   DJI motors over CAN: M3508 (C620 ESC), M2006 (C610 ESC), GM6020.
 *
 *          - Receive: each registered motor gets its feedback frame through bsp_can; the frame
 *            is decoded in the CAN RX interrupt into motor->measure.
 *          - Transmit: DJI packs up to 4 motors into one control frame. DJIMotor_SetOutput()
 *            only stores the value; DJIMotor_SendAll() packs every motor into its frame
 *            (0x200 / 0x1FF / 0x2FF per bus) and sends each frame that has a motor in it.
 *
 *          Units are the motor's raw units (rotor rpm, encoder ticks, raw current).
 *          Converting to wheel speed or output-shaft angle (gear ratio) is the caller's job.
 *
 *          ID rules (from the ESC manuals):
 *            C620 / C610, ESC id 1..8: control 0x200 (id 1-4) / 0x1FF (id 5-8), feedback 0x200 + id
 *            GM6020,      ESC id 1..7: control 0x1FF (id 1-4) / 0x2FF (id 5-7), feedback 0x204 + id
 *            (GM6020 uses voltage control here.)
 *          So on one bus, C620/C610 id 5-8 and GM6020 id 1-4 share both control and feedback IDs.
 *          Register rejects such a conflict.
 *
 * @version beta
 * @date    2026-11-11
 * @copyright Copyright (c) 2026
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"

/* ------------------------------ parameters ------------------------------ */

#define DJI_MOTOR_MAX 12 /* pool size: up to 6 motors per bus on 2 buses */

#define DJI_ECD_RANGE 8192 /* encoder ticks per rotor revolution (0..8191) */

/* ------------------------------ types ------------------------------ */

typedef enum
{
    DJI_MOTOR_M3508 = 0, /* C620 ESC, current control, output range +-16384 (= +-20 A) */
    DJI_MOTOR_M2006,     /* C610 ESC, current control, output range +-10000 (= +-10 A) */
    DJI_MOTOR_GM6020,    /* voltage control */
} DJIMotor_Type_e;

/** What the caller tells the module about one motor. */
typedef struct
{
    DJIMotor_Type_e type;
    CAN_Bus_e bus;  /* which bus the ESC is wired to */
    uint8_t esc_id; /* the ID set on the ESC itself (C620/C610: 1..8, GM6020: 1..7) */
} DJIMotor_Config_t;

/** Decoded feedback. Written in the CAN RX interrupt; read it with DJIMotor_GetMeasure(). */
typedef struct
{
    uint16_t ecd;         /* rotor angle, 0..8191 per revolution */
    int16_t speed_rpm;    /* rotor speed in rpm, signed (negative = reverse) */
    int16_t current_raw;  /* actual current in raw ESC units, signed */
    uint8_t temperature;  /* degrees Celsius */

    int32_t total_rounds; /* full rotor revolutions since start, signed; for multi-turn angle */
    uint32_t rx_count;    /* feedback frames received; if it stops growing, the motor is offline */
} DJIMotor_Measure_t;

/**
 * One motor. Obtained from DJIMotor_Register(); never create one yourself.
 * The struct is visible so the debugger can show it, but callers should only:
 *   - read measure through DJIMotor_GetMeasure(), and
 *   - set output through DJIMotor_SetOutput().
 */
typedef struct
{
    DJIMotor_Config_t config;
    volatile DJIMotor_Measure_t measure; /* written by the RX interrupt */
    int16_t output;                      /* value packed into the next control frame */
    uint32_t clamp_count;                /* times SetOutput had to clamp; growing = saturated */

    /* Filled in by Register; used by SendAll. */
    uint8_t group; /* which control frame on its bus: 0 = 0x200, 1 = 0x1FF, 2 = 0x2FF */
    uint8_t slot;  /* position in that frame: bytes [2*slot, 2*slot+1] */
} DJIMotor_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Take a motor from the pool, work out its CAN IDs, and register its feedback with bsp_can.
 * Call during initialization, after BSP_CAN_Init().
 *
 * @return the motor, or NULL if: the config is invalid, the pool is full,
 *         the IDs conflict with an already registered motor, or bsp_can refused the registration.
 */
DJIMotor_t *DJIMotor_Register(const DJIMotor_Config_t *config);

/**
 * Set the value sent on the next DJIMotor_SendAll(): current for M3508/M2006, voltage for GM6020.
 * Takes int32_t on purpose: a PID result like 40000 would already overflow an int16_t before
 * it could be clamped.
 * Values outside the motor type's range are clamped to the limit and counted in clamp_count.
 * Clamping is the last line of defense; the PID should limit its own output (with anti-windup)
 * so it knows when it is saturated. Does not send anything by itself.
 */
void DJIMotor_SetOutput(DJIMotor_t *motor, int32_t output);

/**
 * Copy the latest feedback into *out as one consistent snapshot.
 * The RX interrupt can't change it halfway through the copy.
 */
void DJIMotor_GetMeasure(const DJIMotor_t *motor, DJIMotor_Measure_t *out);

/**
 * Pack every registered motor's output into its control frame and send each frame
 * that has at least one motor. Call once per control cycle (e.g. every 1 ms).
 *
 * @return CAN_OK if every frame was queued, otherwise the first error from BSP_CAN_Transmit.
 */
CAN_Status_e DJIMotor_SendAll(void);

#endif /* DJI_MOTOR_H */