/**
 * @file    robot_msgs.h
 * @author  eason
 * @brief   Messages exchanged between the applications, and their topics.
 *
 *          Data flow (one board):
 *
 *            DT7 --> robot_cmd --[topic_chassis_cmd]--> chassis --> 4 x M3508
 *                       ^                                  |
 *                       +--[topic_gimbal_feedback]-- gimbal (yaw GM6020)
 *                                                          |
 *                                    [topic_chassis_feedback] (odometry, for debugging / nav)
 *
 *          Units are SI everywhere: m/s, rad/s, rad. Frame: x forward, y left, CCW positive.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ROBOT_MSGS_H
#define ROBOT_MSGS_H

#include <stdbool.h>
#include <stdint.h>
#include "msg_center.h"

/* ------------------------------ chassis command ------------------------------ */

typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0, /* all wheel currents 0, PIDs reset (e-stop, remote lost) */
    CHASSIS_MODE_GIMBAL_FRAME,   /* vx, vy are in the gimbal frame: "forward" = where the gimbal looks */
    CHASSIS_MODE_CHASSIS_FRAME,  /* vx, vy are in the chassis frame (navigation sends this) */
} Chassis_Mode_e;

typedef struct
{
    Chassis_Mode_e mode;
    float vx;              /* m/s, in the frame chosen by mode */
    float vy;              /* m/s */
    float wz;              /* rad/s, chassis rotation */
    float yaw_offset_rad;  /* gimbal yaw relative to the chassis, CCW positive, (-pi, pi]; used by GIMBAL_FRAME */
} Chassis_Cmd_t;

/* ------------------------------ gimbal feedback ------------------------------ */

typedef struct
{
    bool yaw_online;       /* the yaw motor sent feedback recently */
    float yaw_offset_rad;  /* gimbal yaw relative to the chassis, CCW positive, (-pi, pi] */
} Gimbal_Feedback_t;

/* ------------------------------ chassis feedback ------------------------------ */

typedef struct
{
    Chassis_Mode_e mode;   /* the mode the chassis is actually running (ZERO_FORCE after a timeout) */
    float vx;              /* m/s, measured from the wheels, chassis frame */
    float vy;
    float wz;              /* rad/s */
} Chassis_Feedback_t;

/* ------------------------------ topics ------------------------------ */

MSG_TOPIC_DECLARE(topic_chassis_cmd);      /* published by robot_cmd */
MSG_TOPIC_DECLARE(topic_gimbal_feedback);  /* published by gimbal */
MSG_TOPIC_DECLARE(topic_chassis_feedback); /* published by chassis */

#endif /* ROBOT_MSGS_H */