/**
 * @file    chassis.h
 * @author  eason
 * @brief   Chassis application: topic_chassis_cmd -> four omni wheels; odometry -> topic_chassis_feedback.
 *
 *          Each cycle:
 *          1. read the command; missing or older than CHASSIS_CMD_TIMEOUT_MS -> ZERO_FORCE
 *          2. GIMBAL_FRAME: rotate (vx, vy) by the yaw offset into the chassis frame
 *          3. limit acceleration per axis (ramp)
 *          4. inverse kinematics -> rotor rpm targets; if one exceeds CHASSIS_MAX_ROTOR_RPM,
 *             scale ALL four down by the same factor (keeps the direction of motion)
 *          5. one speed PID per wheel -> current
 *          6. forward kinematics on the measured rpm -> topic_chassis_feedback
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef CHASSIS_H
#define CHASSIS_H

void Chassis_Init(void);
void Chassis_Task(float dt_s);

#endif /* CHASSIS_H */