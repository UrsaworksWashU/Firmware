/**
 * @file    pid.c
 * @author  eason
 * @brief   Positional PID controller. See pid.h for the interface and the protections.
 *
 *          Pure math: no HAL, no bsp, no interrupts. It can be compiled and tested on a PC.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "pid.h"
#include <stddef.h>

/* ------------------------------ private helpers ------------------------------ */

/** Limit value to [-limit, +limit]. limit must be >= 0. */
static float Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/**
 * Check the config ranges.
 * Each test is written as !(good condition) so that NaN fails it:
 * every comparison with NaN is false, so "x <= 0" would let NaN through but "!(x > 0)" does not.
 */
static bool ConfigValid(const PID_Config_t *config)
{
    if (!(config->output_max > 0.0f))
    {
        return false;
    }
    if (!(config->i_out_max >= 0.0f))
    {
        return false;
    }
    if (!(config->d_lpf_alpha >= 0.0f && config->d_lpf_alpha < 1.0f))
    {
        return false;
    }
    return true;
}

/* ------------------------------ public functions ------------------------------ */

bool PID_Init(PID_t *pid, const PID_Config_t *config)
{
    if (pid == NULL || config == NULL || !ConfigValid(config))
    {
        return false;
    }

    pid->config = *config;
    pid->saturate_count = 0;
    PID_Reset(pid);
    return true;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->last_measure = 0.0f;
    pid->has_last = false;

    pid->error = 0.0f;
    pid->p_out = 0.0f;
    pid->output = 0.0f;
    /* saturate_count is a statistic: kept across Reset, cleared only by Init. */
}

float PID_Calc(PID_t *pid, float target, float measure, float dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    /* dt == 0 would divide by zero in the D term; NaN dt would poison every term.
     * !(dt > 0) rejects both. */
    if (!(dt > 0.0f))
    {
        return pid->output;
    }

    const PID_Config_t *config = &pid->config;
    const float error = target - measure;

    /* ---- P ---- */
    const float p_out = config->kp * error;

    /* ---- D: on the measurement, so a step in the target gives no spike ----
     * d(error)/dt = d(target)/dt - d(measure)/dt. Dropping the target part leaves -d(measure)/dt. */
    float d_out = 0.0f;
    if (pid->has_last)
    {
        const float d_raw = -config->kd * (measure - pid->last_measure) / dt;
        /* First-order low-pass: keep alpha of the old value, take (1 - alpha) of the new one. */
        d_out = config->d_lpf_alpha * pid->d_out + (1.0f - config->d_lpf_alpha) * d_raw;
    }

    /* ---- I: accumulate in output units, then apply the two limits ---- */
    const float i_step = config->ki * error * dt;
    float i_out = Clamp(pid->i_out + i_step, config->i_out_max);

    /* Anti-windup: if the output would be saturated AND this step pushes further into the
     * saturation, reject the step. A step in the opposite direction is still allowed,
     * so the integral can unwind as soon as the error changes sign. */
    const float unclamped = p_out + i_out + d_out;
    const bool winding_up = (unclamped > config->output_max) && (i_step > 0.0f);
    const bool winding_down = (unclamped < -config->output_max) && (i_step < 0.0f);
    if (winding_up || winding_down)
    {
        /* Re-clamp the old value too: i_out_max may have been lowered live in the debugger. */
        i_out = Clamp(pid->i_out, config->i_out_max);
    }

    /* ---- Sum and final clamp ---- */
    float output = p_out + i_out + d_out;
    if (output > config->output_max || output < -config->output_max)
    {
        pid->saturate_count++;
        output = Clamp(output, config->output_max);
    }

    /* ---- Save state for the next call and for Ozone ---- */
    pid->i_out = i_out;
    pid->d_out = d_out;
    pid->last_measure = measure;
    pid->has_last = true;

    pid->error = error;
    pid->p_out = p_out;
    pid->output = output;

    return output;
}