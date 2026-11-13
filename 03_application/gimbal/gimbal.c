/**
 * @file    gimbal.c
 * @author  eason
 * @brief   Gimbal application: yaw motor feedback -> topic_gimbal_feedback. No control yet.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "gimbal.h"
#include "robot_config.h"
#include "robot_msgs.h"
#include "dji_motor.h"
#include "bsp_time.h"
#include <stddef.h>

#define GIMBAL_PI 3.14159265359f

MSG_TOPIC_DEFINE(topic_gimbal_feedback, Gimbal_Feedback_t);

/* File-static state, all visible in Ozone. */
static DJIMotor_t *yaw_motor;            /* NULL: registration failed (ID conflict?) */
static DJIMotor_Measure_t yaw_measure;
static uint32_t yaw_last_rx_count;
static uint32_t yaw_last_rx_ms;
static bool yaw_ever_received;
static Gimbal_Feedback_t gimbal_feedback; /* the message published each cycle */

/** Encoder ticks away from the aligned position -> radians in (-pi, pi], CCW positive. */
static float YawOffsetRad(uint16_t ecd)
{
    int32_t diff = (int32_t)ecd - (int32_t)GIMBAL_YAW_ALIGN_ECD;

    /* The encoder wraps at 8192: take the shorter way around, so the result is within half a turn. */
    if (diff > DJI_ECD_RANGE / 2)
    {
        diff -= DJI_ECD_RANGE;
    }
    else if (diff <= -DJI_ECD_RANGE / 2)
    {
        diff += DJI_ECD_RANGE;
    }
    return (float)(GIMBAL_YAW_DIRECTION * diff) * (2.0f * GIMBAL_PI / (float)DJI_ECD_RANGE);
}

void Gimbal_Init(void)
{
    const DJIMotor_Config_t yaw_cfg = {
        .type = DJI_MOTOR_GM6020,
        .bus = GIMBAL_YAW_BUS,
        .esc_id = GIMBAL_YAW_ESC_ID,
    };
    yaw_motor = DJIMotor_Register(&yaw_cfg);
}

void Gimbal_Task(float dt_s)
{
    (void)dt_s; /* used once yaw control exists */

    bool online = false;
    float offset = 0.0f;

    if (yaw_motor != NULL)
    {
        DJIMotor_GetMeasure(yaw_motor, &yaw_measure);

        /* Online = rx_count moved within the timeout. */
        const uint32_t now = BSP_Time_GetMs();
        if (yaw_measure.rx_count != yaw_last_rx_count)
        {
            yaw_last_rx_count = yaw_measure.rx_count;
            yaw_last_rx_ms = now;
            yaw_ever_received = true;
        }
        online = yaw_ever_received && (now - yaw_last_rx_ms) < GIMBAL_YAW_TIMEOUT_MS;

        if (online)
        {
            offset = YawOffsetRad(yaw_measure.ecd);
        }

        /* Zero force: the gimbal can be turned by hand. Gimbal control comes later. */
        DJIMotor_SetOutput(yaw_motor, 0);
    }

    gimbal_feedback.yaw_online = online;
    gimbal_feedback.yaw_offset_rad = offset;
    (void)Msg_Publish(&topic_gimbal_feedback, &gimbal_feedback, sizeof(gimbal_feedback));
}