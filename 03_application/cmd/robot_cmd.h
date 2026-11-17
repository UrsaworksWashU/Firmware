/**
 * @file    robot_cmd.h
 * @author  eason
 * @brief   Command application: turns operator input (DT7) into commands for the other apps.
 *
 *          DT7 mapping (one-board infantry, chassis only for now):
 *            left switch DOWN  -> ZERO_FORCE
 *            left switch MID   -> GIMBAL_FRAME driven by the sticks (needs the yaw motor online)
 *                                   left stick    -> vx (up = forward), vy (left = +y)
 *                                   right stick x -> wz (left = counter-clockwise)
 *            left switch UP    -> GIMBAL_FRAME driven by navigation (needs the yaw motor online)
 *                                   nav vx, vy (barrel frame); wz = 0, spin flag ignored for now.
 *                                   Nav silent for NAV_CMD_TIMEOUT_MS -> speed 0, wheels hold.
 *          Remote offline -> ZERO_FORCE. The remote always wins: navigation only drives in UP.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

void RobotCmd_Init(void);
void RobotCmd_Task(float dt_s);

#endif /* ROBOT_CMD_H */