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
#include "nav_cmd.h"

MSG_TOPIC_DEFINE(topic_chassis_cmd, Chassis_Cmd_t);

/** Who is driving the chassis this cycle. */
typedef enum
{
    CMD_SOURCE_NONE = 0, /* zero force */
    CMD_SOURCE_RC,
    CMD_SOURCE_NAV,
} Cmd_Source_e;

/* File-static state, all visible in Ozone. */
static Msg_Sub_t gimbal_sub;
static DT7_Data_t rc;
static bool rc_online;
static NavCmd_Data_t nav;
static bool nav_online;
static Cmd_Source_e source;
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
    rc_online = DT7_GetData(&rc);     /* offline: rc is all zeros */
    nav_online = NavCmd_GetData(&nav); /* offline (silent > 300 ms): nav is all zeros */

    uint32_t gimbal_age_ms = 0;
    const Msg_Status_e gimbal_status = Msg_Read(&gimbal_sub, &gimbal_feedback, sizeof(gimbal_feedback), &gimbal_age_ms);
    const bool gimbal_fresh = (gimbal_status == MSG_NEW || gimbal_status == MSG_OLD) &&
                              gimbal_age_ms < GIMBAL_YAW_TIMEOUT_MS;
    gimbal_ok = gimbal_fresh && gimbal_feedback.yaw_online;

    /* ---- Source ----
     * Both RC and navigation speak in the gimbal (barrel) frame, so both need the yaw angle:
     * without it "forward" is unknown, so refuse to move rather than guess. */
    source = CMD_SOURCE_NONE;
    if (rc_online && gimbal_ok)
    {
        if (rc.switch_left == DT7_SW_MID)
        {
            source = CMD_SOURCE_RC;
        }
        else if (rc.switch_left == DT7_SW_UP)
        {
            source = CMD_SOURCE_NAV;
        }
    }

    /* ---- Mode and velocities ---- */
    chassis_cmd.mode = (source == CMD_SOURCE_NONE) ? CHASSIS_MODE_ZERO_FORCE : CHASSIS_MODE_GIMBAL_FRAME;
    if (source == CMD_SOURCE_RC)
    {
        /* Stick right is positive, but +y is left and +wz is counter-clockwise, hence the minus signs. */
        chassis_cmd.vx = StickToUnit(rc.left_y) * RC_MAX_VXY_MPS;
        chassis_cmd.vy = -StickToUnit(rc.left_x) * RC_MAX_VXY_MPS;
        chassis_cmd.wz = -StickToUnit(rc.right_x) * RC_MAX_WZ_RADPS;
    }
    else if (source == CMD_SOURCE_NAV)
    {
        /* Navigation already uses x forward / y left (REP-103): no sign change.
         * Nav offline -> nav is zeros -> the wheels brake and hold instead of coasting. */
        chassis_cmd.vx = nav.vx_mps;
        chassis_cmd.vy = nav.vy_mps;
        chassis_cmd.wz = 0.0f; /* TODO: spin when nav.spin, once the speed is decided */
    }
    else
    {
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        chassis_cmd.wz = 0.0f;
    }
    chassis_cmd.yaw_offset_rad = gimbal_ok ? gimbal_feedback.yaw_offset_rad : 0.0f;

    (void)Msg_Publish(&topic_chassis_cmd, &chassis_cmd, sizeof(chassis_cmd));
}