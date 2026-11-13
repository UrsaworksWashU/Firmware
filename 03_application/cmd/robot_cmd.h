/**
 * @file    robot_cmd.h
 * @author  eason
 * @brief   Command application: turns operator input (DT7) into commands for the other apps.
 *
 *          DT7 mapping (one-board infantry, chassis only for now):
 *            left switch DOWN  -> CHASSIS_MODE_ZERO_FORCE
 *            left switch MID   -> CHASSIS_MODE_GIMBAL_FRAME (needs the yaw motor online)
 *            left switch UP    -> CHASSIS_MODE_CHASSIS_FRAME
 *            left stick        -> vx (up = forward), vy (left = +y)
 *            right stick x     -> wz (left = counter-clockwise)
 *          Remote offline -> ZERO_FORCE.
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