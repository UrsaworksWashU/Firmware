/**
 * @file    pid.h
 * @author  eason
 * @brief   Positional PID controller in single-precision float (the F407 has a hardware FPU).
 *
 *          output = P + I + D, where
 *            P = kp * error
 *            I = sum of (ki * error * dt), stored directly in output units (i_out)
 *            D = -kd * d(measure)/dt   (derivative on measurement, optionally low-pass filtered)
 *
 *          Protections:
 *          - |output| <= output_max, |i_out| <= i_out_max.
 *          - Anti-windup: while the output is saturated, the integral does not grow further
 *            in the saturated direction (it may still shrink).
 *          - Derivative on measurement: a step in the target does not cause a derivative spike.
 *          - The first call after Init/Reset skips the D term (there is no previous measurement yet).
 *
 *          Usage: the caller owns the PID_t (e.g. inside a chassis struct); there is no registry
 *          and no pool. Not thread-safe: call Calc from one context (one task or one loop) only.
 *          Gains can be changed live through pid.config (e.g. in Ozone) without an output bump,
 *          because the integral is stored as i_out, not as a raw error sum.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------ types ------------------------------ */

typedef struct
{
    float kp;          /* output units per error unit */
    float ki;          /* output units per (error unit * second) */
    float kd;          /* output units per (error unit / second) */
    float output_max;  /* |output| limit, must be > 0 */
    float i_out_max;   /* |I term| limit in output units, must be >= 0 (0 disables I) */
    float d_lpf_alpha; /* derivative low-pass, [0, 1): 0 = no filter, closer to 1 = smoother but laggier */
} PID_Config_t;

typedef struct
{
    PID_Config_t config; /* copied at Init; may be edited live */

    /* Internal state */
    float i_out;         /* accumulated I term, already in output units */
    float d_out;         /* filtered D term from the last call */
    float last_measure;  /* for derivative on measurement */
    bool has_last;       /* false right after Init/Reset: D is skipped on the next call */

    /* Last-call values, for watching in Ozone */
    float error;
    float p_out;
    float output;
    uint32_t saturate_count; /* number of calls where the output hit output_max */
} PID_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Copy the config and clear the state.
 * @return false (and leaves pid untouched) if pid/config is NULL or the config is invalid.
 */
bool PID_Init(PID_t *pid, const PID_Config_t *config);

/**
 * Clear the integral and derivative history, keep the config.
 * Call when the controlled device was disabled or the control mode changed,
 * so stale integral from the old situation does not kick in.
 */
void PID_Reset(PID_t *pid);

/**
 * Run one controller step.
 * @param target   desired value (same unit as measure)
 * @param measure  measured value
 * @param dt       seconds since the previous call, must be > 0
 * @return the new output, clamped to +/- output_max.
 *         If dt <= 0 the state is not changed and the previous output is returned.
 */
float PID_Calc(PID_t *pid, float target, float measure, float dt);

#endif /* PID_H */