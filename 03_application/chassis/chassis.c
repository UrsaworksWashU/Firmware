/**
 * @file    chassis.c
 * @author  eason
 * @brief   Chassis application. See chassis.h for the per-cycle steps.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "chassis.h"
#include "robot_config.h"
#include "robot_msgs.h"
#include "dji_motor.h"
#include "omni.h"
#include "pid.h"
#include <math.h>
#include <stddef.h>

#define CHASSIS_DEG_TO_RAD 0.01745329252f

MSG_TOPIC_DEFINE(topic_chassis_feedback, Chassis_Feedback_t);

/* Wheel index order everywhere: LF, RF, LB, RB. */
enum
{
    WHEEL_LF = 0,
    WHEEL_RF,
    WHEEL_LB,
    WHEEL_RB,
};

/* Everything worth watching in Ozone, in one struct: add "chassis" to Live Watch. */
typedef struct
{
    bool omni_ok;                          /* false: geometry config rejected, chassis stays at zero force */
    Chassis_Mode_e mode;                   /* mode actually running */
    uint32_t cmd_age_ms;                   /* how old the latest command is */
    Chassis_Cmd_t cmd;                     /* latest command as received */

    float vx_target, vy_target, wz_target; /* chassis frame, before the ramp */
    float vx_ramped, vy_ramped, wz_ramped; /* after the acceleration limit */
    float speed_scale;                     /* 1 = no rpm limit hit, < 1 = scaled down */

    float target_rpm[OMNI_WHEEL_NUM];
    float measured_rpm[OMNI_WHEEL_NUM];
    float current_out[OMNI_WHEEL_NUM];
} Chassis_State_t;

static Chassis_State_t chassis;
static Omni_t omni;
static DJIMotor_t *wheel_motor[OMNI_WHEEL_NUM]; /* NULL: registration failed */
static PID_t wheel_pid[OMNI_WHEEL_NUM];
static Msg_Sub_t cmd_sub;
static Chassis_Feedback_t feedback;

/* ------------------------------ private helpers ------------------------------ */

/** Move current toward target by at most max_step. */
static float Ramp(float current, float target, float max_step)
{
    const float diff = target - current;
    if (diff > max_step)
    {
        return current + max_step;
    }
    if (diff < -max_step)
    {
        return current - max_step;
    }
    return target;
}

/** Read every wheel's rotor rpm (0 for a wheel that failed to register). */
static void ReadWheels(void)
{
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        chassis.measured_rpm[i] = 0.0f;
        if (wheel_motor[i] != NULL)
        {
            DJIMotor_Measure_t measure;
            DJIMotor_GetMeasure(wheel_motor[i], &measure);
            chassis.measured_rpm[i] = (float)measure.speed_rpm;
        }
    }
}

/** Zero force: currents 0, PIDs and ramps cleared so re-enabling starts from rest. */
static void StopAll(void)
{
    chassis.vx_ramped = 0.0f;
    chassis.vy_ramped = 0.0f;
    chassis.wz_ramped = 0.0f;
    chassis.speed_scale = 1.0f;
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        PID_Reset(&wheel_pid[i]);
        chassis.target_rpm[i] = 0.0f;
        chassis.current_out[i] = 0.0f;
        if (wheel_motor[i] != NULL)
        {
            DJIMotor_SetOutput(wheel_motor[i], 0);
        }
    }
}

/* ------------------------------ public functions ------------------------------ */

void Chassis_Init(void)
{
    /* ---- Kinematics ---- */
    const float b = CHASSIS_HALF_WHEELBASE_M;
    const float a = CHASSIS_HALF_TRACK_M;
    const Omni_Config_t omni_cfg = {
        .wheel = {
            [WHEEL_LF] = {.x_m = b, .y_m = a, .drive_angle_rad = CHASSIS_DRIVE_ANGLE_LF_DEG * CHASSIS_DEG_TO_RAD},
            [WHEEL_RF] = {.x_m = b, .y_m = -a, .drive_angle_rad = CHASSIS_DRIVE_ANGLE_RF_DEG * CHASSIS_DEG_TO_RAD},
            [WHEEL_LB] = {.x_m = -b, .y_m = a, .drive_angle_rad = CHASSIS_DRIVE_ANGLE_LB_DEG * CHASSIS_DEG_TO_RAD},
            [WHEEL_RB] = {.x_m = -b, .y_m = -a, .drive_angle_rad = CHASSIS_DRIVE_ANGLE_RB_DEG * CHASSIS_DEG_TO_RAD},
        },
        .wheel_radius_m = CHASSIS_WHEEL_RADIUS_M,
        .gear_ratio = CHASSIS_GEAR_RATIO,
    };
    chassis.omni_ok = Omni_Init(&omni, &omni_cfg);

    /* ---- Motors and speed loops ---- */
    const uint8_t esc_id[OMNI_WHEEL_NUM] = {
        [WHEEL_LF] = CHASSIS_ESC_ID_LF,
        [WHEEL_RF] = CHASSIS_ESC_ID_RF,
        [WHEEL_LB] = CHASSIS_ESC_ID_LB,
        [WHEEL_RB] = CHASSIS_ESC_ID_RB,
    };
    const PID_Config_t pid_cfg = {
        .kp = CHASSIS_WHEEL_KP,
        .ki = CHASSIS_WHEEL_KI,
        .kd = 0.0f,
        .output_max = CHASSIS_WHEEL_OUT_MAX,
        .i_out_max = CHASSIS_WHEEL_I_MAX,
        .d_lpf_alpha = 0.0f,
    };
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        const DJIMotor_Config_t motor_cfg = {
            .type = DJI_MOTOR_M3508,
            .bus = CHASSIS_MOTOR_BUS,
            .esc_id = esc_id[i],
        };
        wheel_motor[i] = DJIMotor_Register(&motor_cfg);
        (void)PID_Init(&wheel_pid[i], &pid_cfg); /* constant config, checked at compile-test time */
    }

    Msg_Subscribe(&cmd_sub, &topic_chassis_cmd);
    chassis.mode = CHASSIS_MODE_ZERO_FORCE;
    chassis.speed_scale = 1.0f;
}

void Chassis_Task(float dt_s)
{
    /* ---- 1. Command ---- */
    const Msg_Status_e status = Msg_Read(&cmd_sub, &chassis.cmd, sizeof(chassis.cmd), &chassis.cmd_age_ms);
    const bool cmd_fresh = (status == MSG_NEW || status == MSG_OLD) &&
                           chassis.cmd_age_ms < CHASSIS_CMD_TIMEOUT_MS;
    chassis.mode = (cmd_fresh && chassis.omni_ok) ? chassis.cmd.mode : CHASSIS_MODE_ZERO_FORCE;

    ReadWheels();

    if (chassis.mode == CHASSIS_MODE_ZERO_FORCE)
    {
        chassis.vx_target = 0.0f;
        chassis.vy_target = 0.0f;
        chassis.wz_target = 0.0f;
        StopAll();
    }
    else
    {
        /* ---- 2. Frame ----
         * The gimbal is turned by theta (CCW) from the chassis. A vector given in the gimbal frame
         * is the same vector rotated by +theta in the chassis frame:
         *   theta = +90 deg, "forward" (1, 0) -> (0, 1) = chassis left. */
        if (chassis.mode == CHASSIS_MODE_GIMBAL_FRAME)
        {
            const float c = cosf(chassis.cmd.yaw_offset_rad);
            const float s = sinf(chassis.cmd.yaw_offset_rad);
            chassis.vx_target = chassis.cmd.vx * c - chassis.cmd.vy * s;
            chassis.vy_target = chassis.cmd.vx * s + chassis.cmd.vy * c;
        }
        else
        {
            chassis.vx_target = chassis.cmd.vx;
            chassis.vy_target = chassis.cmd.vy;
        }
        chassis.wz_target = chassis.cmd.wz;

        /* ---- 3. Acceleration limit ---- */
        chassis.vx_ramped = Ramp(chassis.vx_ramped, chassis.vx_target, CHASSIS_MAX_ACCEL_XY * dt_s);
        chassis.vy_ramped = Ramp(chassis.vy_ramped, chassis.vy_target, CHASSIS_MAX_ACCEL_XY * dt_s);
        chassis.wz_ramped = Ramp(chassis.wz_ramped, chassis.wz_target, CHASSIS_MAX_ACCEL_WZ * dt_s);

        /* ---- 4. Inverse kinematics + common scaling ---- */
        Omni_Inverse(&omni, chassis.vx_ramped, chassis.vy_ramped, chassis.wz_ramped, chassis.target_rpm);

        float max_abs = 0.0f;
        for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            const float abs_rpm = fabsf(chassis.target_rpm[i]);
            if (abs_rpm > max_abs)
            {
                max_abs = abs_rpm;
            }
        }
        chassis.speed_scale = (max_abs > CHASSIS_MAX_ROTOR_RPM) ? (CHASSIS_MAX_ROTOR_RPM / max_abs) : 1.0f;

        /* ---- 5. Wheel speed loops ---- */
        for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            chassis.target_rpm[i] *= chassis.speed_scale;
            chassis.current_out[i] = PID_Calc(&wheel_pid[i], chassis.target_rpm[i], chassis.measured_rpm[i], dt_s);
            if (wheel_motor[i] != NULL)
            {
                DJIMotor_SetOutput(wheel_motor[i], (int32_t)chassis.current_out[i]);
            }
        }
    }

    /* ---- 6. Odometry ---- */
    feedback.mode = chassis.mode;
    Omni_Forward(&omni, chassis.measured_rpm, &feedback.vx, &feedback.vy, &feedback.wz);
    (void)Msg_Publish(&topic_chassis_feedback, &feedback, sizeof(feedback));
}