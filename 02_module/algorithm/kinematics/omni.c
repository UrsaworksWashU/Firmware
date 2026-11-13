/**
 * @file    omni.c
 * @author  eason
 * @brief   Omni wheel kinematics. See omni.h for the derivation.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "omni.h"
#include <math.h>
#include <stddef.h>

#define OMNI_TWO_PI 6.28318530718f

bool Omni_Init(Omni_t *omni, const Omni_Config_t *config)
{
    if (omni == NULL || config == NULL ||
        !(config->wheel_radius_m > 0.0f) || !(config->gear_ratio > 0.0f))
    {
        return false;
    }

    /* Wheel surface speed (m/s) -> rotor rpm:
     * wheel rev/s = v / (2*pi*r), wheel rpm = that * 60, rotor rpm = wheel rpm * gear_ratio. */
    const float rpm_per_mps = 60.0f / (OMNI_TWO_PI * config->wheel_radius_m) * config->gear_ratio;

    /* ---- Inverse: one row of J per wheel ---- */
    float j[OMNI_WHEEL_NUM][3];
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        const Omni_Wheel_t *w = &config->wheel[i];
        const float c = cosf(w->drive_angle_rad);
        const float s = sinf(w->drive_angle_rad);
        j[i][0] = rpm_per_mps * c;
        j[i][1] = rpm_per_mps * s;
        j[i][2] = rpm_per_mps * (w->x_m * s - w->y_m * c);
    }

    /* ---- Forward: (J^T J)^-1 J^T ---- */
    float a[3][3] = {{0}}; /* A = J^T J, 3 x 3, symmetric */
    for (uint32_t r = 0; r < 3; r++)
    {
        for (uint32_t c = 0; c < 3; c++)
        {
            for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
            {
                a[r][c] += j[i][r] * j[i][c];
            }
        }
    }

    /* Inverse of a 3 x 3 by cofactors: inv = adj(A) / det(A). */
    const float c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const float c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const float c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const float det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;

    /* Singular (or nearly): the wheels cannot tell some motion apart, e.g. all wheels parallel.
     * Compare against the matrix's own scale, since the entries are large (rpm per m/s, squared). */
    const float scale = (a[0][0] + a[1][1] + a[2][2]) / 3.0f;
    if (!(fabsf(det) > 1e-6f * scale * scale * scale))
    {
        return false;
    }

    float inv[3][3];
    inv[0][0] = c00 / det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
    inv[1][0] = c01 / det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det;
    inv[2][0] = c02 / det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;

    /* Store only after every check passed, so a failed Init leaves *omni untouched. */
    omni->config = *config;
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        for (uint32_t k = 0; k < 3; k++)
        {
            omni->inverse[i][k] = j[i][k];
        }
    }
    for (uint32_t r = 0; r < 3; r++)
    {
        for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            omni->forward[r][i] = inv[r][0] * j[i][0] + inv[r][1] * j[i][1] + inv[r][2] * j[i][2];
        }
    }
    return true;
}

void Omni_Inverse(const Omni_t *omni, float vx, float vy, float wz, float rotor_rpm[OMNI_WHEEL_NUM])
{
    if (omni == NULL || rotor_rpm == NULL)
    {
        return;
    }
    for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        rotor_rpm[i] = omni->inverse[i][0] * vx + omni->inverse[i][1] * vy + omni->inverse[i][2] * wz;
    }
}

void Omni_Forward(const Omni_t *omni, const float rotor_rpm[OMNI_WHEEL_NUM], float *vx, float *vy, float *wz)
{
    if (omni == NULL || rotor_rpm == NULL || vx == NULL || vy == NULL || wz == NULL)
    {
        return;
    }
    float result[3] = {0};
    for (uint32_t r = 0; r < 3; r++)
    {
        for (uint32_t i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            result[r] += omni->forward[r][i] * rotor_rpm[i];
        }
    }
    *vx = result[0];
    *vy = result[1];
    *wz = result[2];
}