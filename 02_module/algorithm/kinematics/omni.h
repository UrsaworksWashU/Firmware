/**
 * @file    omni.h
 * @author  eason
 * @brief   Kinematics for a chassis with N omni wheels at arbitrary positions and angles.
 *
 *          Frame (same as ROS REP-103): x forward, y left, z up; wz > 0 is counter-clockwise
 *          seen from above. Units: m, m/s, rad/s, rotor rpm.
 *
 *          A wheel at position r = (x, y) whose rotor, turning positive, pushes the chassis along
 *          the unit vector d = (cos a, sin a) must roll at the chassis velocity at r projected on d:
 *
 *              v_wheel = d . (v + w x r)
 *                      = cos(a) * vx + sin(a) * vy + (x * sin(a) - y * cos(a)) * wz
 *
 *              rotor_rpm = v_wheel / radius * 60 / (2*pi) * gear_ratio
 *
 *          Stacking one row per wheel gives v_wheels = J * (vx, vy, wz), J is N x 3 (inverse kinematics).
 *          Forward kinematics (wheel odometry) solves it back in the least-squares sense:
 *          (vx, vy, wz) = (J^T J)^-1 J^T * v_wheels. With 4 wheels and 3 unknowns, a wheel that
 *          slips shows up as a residual instead of corrupting one axis.
 *          Omni_Init computes both matrices once.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef OMNI_H
#define OMNI_H

#include <stdbool.h>
#include <stdint.h>

#define OMNI_WHEEL_NUM 4u

/* ------------------------------ types ------------------------------ */

typedef struct
{
    float x_m;             /* wheel contact point, chassis frame */
    float y_m;
    float drive_angle_rad; /* direction the chassis is pushed when the rotor turns positive, CCW from +x */
} Omni_Wheel_t;

typedef struct
{
    Omni_Wheel_t wheel[OMNI_WHEEL_NUM];
    float wheel_radius_m;
    float gear_ratio;      /* rotor turns per wheel turn, e.g. 3591/187 for a stock M3508 */
} Omni_Config_t;

typedef struct
{
    Omni_Config_t config;
    float inverse[OMNI_WHEEL_NUM][3]; /* J, already scaled to rotor rpm */
    float forward[3][OMNI_WHEEL_NUM]; /* (J^T J)^-1 J^T, from rotor rpm back to (vx, vy, wz) */
} Omni_t;

/* ------------------------------ functions ------------------------------ */

/**
 * Copy the config and precompute both matrices.
 * @return false if the config is invalid (radius or ratio <= 0) or the wheels cannot
 *         determine all three velocities (J^T J singular, e.g. all wheels parallel).
 */
bool Omni_Init(Omni_t *omni, const Omni_Config_t *config);

/** (vx, vy, wz) -> rotor rpm target for each wheel. */
void Omni_Inverse(const Omni_t *omni, float vx, float vy, float wz, float rotor_rpm[OMNI_WHEEL_NUM]);

/** Measured rotor rpm of each wheel -> (vx, vy, wz) of the chassis. */
void Omni_Forward(const Omni_t *omni, const float rotor_rpm[OMNI_WHEEL_NUM], float *vx, float *vy, float *wz);

#endif /* OMNI_H */