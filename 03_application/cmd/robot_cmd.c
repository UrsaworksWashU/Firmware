/**
 * @file    robot_cmd.c
 * @author  eason
 * @brief   Command application. See robot_cmd.h for the remote mapping.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "robot_cmd.h"
#include "robot_config.h"
#include "robot_msgs.h"
#include "dt7.h"

MSG_TOPIC_DEFINE(topic_chassis_cmd, Chassis_Cmd_t);

/* File-static state, all visible in Ozone. */
static Msg_Sub_t gimbal_sub;
static DT7_Data_t rc;
static bool rc_online;
static Gimbal_Feedback_t gimbal_feedback;
static bool gimbal_ok;                 /* fresh feedback AND yaw motor online */
static Chassis_Cmd_t chassis_cmd;      /* the message published each cycle */

/** Raw stick (-660..660) -> -1..1, with a small dead band around center. */
static float StickToUnit(int16_t raw)
{
    if (raw > -RC_STICK_DEADBAND && raw < RC_STICK_DEADBAND)
    {
        return 0.0f;
    }
    float unit = (float)raw / (float)DT7_CH_MAX_ABS;
    if (unit > 1.0f)
    {
        unit = 1.0f;
    }
    else if (unit < -1.0f)
    {
        unit = -1.0f;
    }
    return unit;
}

void RobotCmd_Init(void)
{
    Msg_Subscribe(&gimbal_sub, &topic_gimbal_feedback);
}

void RobotCmd_Task(float dt_s)
{
    (void)dt_s;

    /* ---- Inputs ---- */
    rc_online = DT7_GetData(&rc); /* offline: rc is all zeros */

    uint32_t gimbal_age_ms = 0;
    const Msg_Status_e gimbal_status = Msg_Read(&gimbal_sub, &gimbal_feedback, sizeof(gimbal_feedback), &gimbal_age_ms);
    const bool gimbal_fresh = (gimbal_status == MSG_NEW || gimbal_status == MSG_OLD) &&
                              gimbal_age_ms < GIMBAL_YAW_TIMEOUT_MS;
    gimbal_ok = gimbal_fresh && gimbal_feedback.yaw_online;

    /* ---- Mode ---- */
    Chassis_Mode_e mode = CHASSIS_MODE_ZERO_FORCE;
    if (rc_online)
    {
        if (rc.switch_left == DT7_SW_MID)
        {
            /* Without the yaw angle, "forward" is unknown: refuse to move rather than guess. */
            mode = gimbal_ok ? CHASSIS_MODE_GIMBAL_FRAME : CHASSIS_MODE_ZERO_FORCE;
        }
        else if (rc.switch_left == DT7_SW_UP)
        {
            mode = CHASSIS_MODE_CHASSIS_FRAME;
        }
    }

    /* ---- Velocities ----
     * Stick right is positive, but +y is left and +wz is counter-clockwise, hence the minus signs. */
    chassis_cmd.mode = mode;
    if (mode == CHASSIS_MODE_ZERO_FORCE)
    {
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        chassis_cmd.wz = 0.0f;
    }
    else
    {
        chassis_cmd.vx = StickToUnit(rc.left_y) * RC_MAX_VXY_MPS;
        chassis_cmd.vy = -StickToUnit(rc.left_x) * RC_MAX_VXY_MPS;
        chassis_cmd.wz = -StickToUnit(rc.right_x) * RC_MAX_WZ_RADPS;
    }
    chassis_cmd.yaw_offset_rad = gimbal_ok ? gimbal_feedback.yaw_offset_rad : 0.0f;

    (void)Msg_Publish(&topic_chassis_cmd, &chassis_cmd, sizeof(chassis_cmd));
}