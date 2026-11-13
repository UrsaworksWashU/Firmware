/**
 * @file    robot_config.h
 * @author  eason
 * @brief   Everything specific to this robot: wiring, geometry, limits, gains, remote mapping.
 *          Another robot gets another copy of this file (later selected by a CMake preset).
 *
 *          Robot: the infantry that ran the HNU code (application/robot_def.h, chassis.c, gimbal.c).
 *          Values marked TODO were never verified there and must be measured on the car.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "bsp_can.h"

/* ------------------------------ timing ------------------------------ */

/* main loop: Robot_Task then HAL_Delay(1), which waits 1..2 ms -> about 2 ms per loop.
 * Replaced by a fixed FreeRTOS period later. */
#define ROBOT_TASK_PERIOD_S 0.002f

/* ------------------------------ chassis wiring ------------------------------ */

/* All four M3508 on CAN1 (HNU chassis.c:103-117). Index order everywhere: LF, RF, LB, RB. */
#define CHASSIS_MOTOR_BUS CAN_BUS_1
#define CHASSIS_ESC_ID_LF 4u
#define CHASSIS_ESC_ID_RF 3u
#define CHASSIS_ESC_ID_LB 1u
#define CHASSIS_ESC_ID_RB 2u

/* ------------------------------ chassis geometry ------------------------------ */

/* TODO measure: HNU says wheelbase 350 mm, track 300 mm, but those numbers were only used
 * inside a hand-tuned rotation factor, never checked. Distances between wheel contact points. */
#define CHASSIS_HALF_WHEELBASE_M 0.150f /* front-back, center to wheel */
#define CHASSIS_HALF_TRACK_M     0.150f /* left-right, center to wheel */

/* TODO measure: HNU's RADIUS_WHEEL (60 mm) was never used anywhere. */
#define CHASSIS_WHEEL_RADIUS_M 0.0763f

/* TODO verify: aftermarket gearbox labelled 268/17 (about 15.76), assumed to REPLACE the stock
 * 3591/187 gearbox. Check: turn a wheel 10 turns by hand, ratio = total_rounds change / 10. */
#define CHASSIS_GEAR_RATIO (268.0f / 17.0f)

/* X layout, rollers at 45 degrees. Drive angle = direction the chassis is pushed when the rotor
 * turns positive. Signs match HNU OmniCalculate (rf = vx + vy, lf = -vx + vy, lb = -vx - vy,
 * rb = vx - vy, all +wz). TODO verify on the lifted car: "forward" must roll every wheel forward. */
#define CHASSIS_DRIVE_ANGLE_LF_DEG  135.0f
#define CHASSIS_DRIVE_ANGLE_RF_DEG   45.0f
#define CHASSIS_DRIVE_ANGLE_LB_DEG -135.0f
#define CHASSIS_DRIVE_ANGLE_RB_DEG  -45.0f

/* ------------------------------ chassis limits ------------------------------ */

#define CHASSIS_MAX_ROTOR_RPM   8000.0f /* M3508 tops out near 9000 rotor rpm; keep headroom */
#define CHASSIS_MAX_ACCEL_XY    2.0f    /* m/s^2, per axis */
#define CHASSIS_MAX_ACCEL_WZ    6.0f    /* rad/s^2 */
#define CHASSIS_CMD_TIMEOUT_MS  50u     /* no chassis command for this long -> zero force */

/* Wheel speed loop, rotor rpm -> raw current. kp from the single-motor test (15 buzzed).
 * ki is 0 until tuned on the ground. */
#define CHASSIS_WHEEL_KP        10.0f
#define CHASSIS_WHEEL_KI        0.0f
#define CHASSIS_WHEEL_I_MAX     5000.0f
#define CHASSIS_WHEEL_OUT_MAX   16384.0f

/* ------------------------------ gimbal yaw ------------------------------ */

/* Yaw GM6020 on CAN1, ESC ID 1 (HNU gimbal.c:62-63). On the bench: the spare 6020, ID dial = 1. */
#define GIMBAL_YAW_BUS          CAN_BUS_1
#define GIMBAL_YAW_ESC_ID       1u
#define GIMBAL_YAW_ALIGN_ECD    508    /* encoder value when gimbal and chassis face the same way (old infantry) */
#define GIMBAL_YAW_DIRECTION    1       /* +1 if the encoder count rises when the gimbal turns CCW seen from above. TODO verify */
#define GIMBAL_YAW_TIMEOUT_MS   50u     /* no yaw feedback for this long -> yaw offline */

/* ------------------------------ remote mapping ------------------------------ */

#define RC_MAX_VXY_MPS   1.5f  /* full stick */
#define RC_MAX_WZ_RADPS  3.0f
#define RC_STICK_DEADBAND 10   /* raw units out of 660 */

#endif /* ROBOT_CONFIG_H */