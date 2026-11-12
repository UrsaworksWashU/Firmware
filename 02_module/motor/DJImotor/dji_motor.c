/**
 * @file    dji_motor.c
 * @author  eason
 * @brief   DJI motor driver. See dji_motor.h for the interface and the ID rules.
 *
 * @version beta
 * @date    2026-11-11
 * @copyright Copyright (c) 2026
 *
 */

#include "dji_motor.h"
#include "main.h" /* CMSIS intrinsics: __get_PRIMASK, __disable_irq, __set_PRIMASK */
#include <stddef.h>

/* ------------------------------ protocol constants ------------------------------ */

/* Control frames on each bus. A motor's "group" is an index into this table. */
#define DJI_FRAME_NUM 3
static const uint32_t frame_id[DJI_FRAME_NUM] = {0x200, 0x1FF, 0x2FF};

#define DJI_FRAME_0x200 0
#define DJI_FRAME_0x1FF 1
#define DJI_FRAME_0x2FF 2

/* Output limits per motor type, in raw ESC units.
 * GM6020: newer manuals give +-25000 for voltage control, older ones +-30000; use the safe value. */
static int32_t OutputLimit(DJIMotor_Type_e type)
{
    switch (type)
    {
    case DJI_MOTOR_M3508:  return 16384;
    case DJI_MOTOR_M2006:  return 10000;
    case DJI_MOTOR_GM6020: return 25000;
    default:               return 0; /* unknown type: allow nothing */
    }
}

/* ------------------------------ private state ------------------------------ */

static DJIMotor_t motor_pool[DJI_MOTOR_MAX];
static uint8_t motor_count; /* entries [0, motor_count) are registered */

/* Which control frames have at least one motor, so SendAll skips empty frames. */
static uint8_t frame_used[CAN_BUS_NUM][DJI_FRAME_NUM];

/* ------------------------------ private helpers ------------------------------ */

/**
 * From type + ESC id, work out the feedback ID and where the motor sits in a control frame.
 * Returns 0 if the ESC id is out of range for this type.
 */
static uint8_t ComputeIds(const DJIMotor_Config_t *config,
                          uint32_t *feedback_id, uint8_t *group, uint8_t *slot)
{
    const uint8_t id = config->esc_id;

    switch (config->type)
    {
    case DJI_MOTOR_M3508:
    case DJI_MOTOR_M2006:
        if (id < 1 || id > 8)
        {
            return 0;
        }
        *feedback_id = 0x200 + id;
        *group = (id <= 4) ? DJI_FRAME_0x200 : DJI_FRAME_0x1FF;
        *slot = (id <= 4) ? (id - 1) : (id - 5);
        return 1;

    case DJI_MOTOR_GM6020:
        if (id < 1 || id > 7)
        {
            return 0;
        }
        *feedback_id = 0x204 + id;
        *group = (id <= 4) ? DJI_FRAME_0x1FF : DJI_FRAME_0x2FF;
        *slot = (id <= 4) ? (id - 1) : (id - 5);
        return 1;

    default:
        return 0;
    }
}

/**
 * bsp_can RX callback for every DJI motor. Runs in the CAN RX interrupt.
 * owner is the DJIMotor_t this feedback ID was registered for.
 *
 * Feedback frame layout (big-endian):
 *   [0..1] rotor angle 0..8191   [2..3] speed rpm (signed)
 *   [4..5] actual current (signed)   [6] temperature   [7] reserved
 */
static void DecodeFeedback(CAN_Bus_e bus, uint32_t id, const uint8_t *data, uint8_t len, void *owner)
{
    (void)bus;
    (void)id;

    if (len < 7)
    {
        return; /* malformed frame: ignore it rather than read garbage */
    }

    DJIMotor_t *motor = (DJIMotor_t *)owner;
    volatile DJIMotor_Measure_t *m = &motor->measure;

    const uint16_t ecd = (uint16_t)((data[0] << 8) | data[1]);

    /* Multi-turn counting. Between two frames (1 ms) the rotor turns far less than half a
     * revolution, so a jump of more than half the range means the angle wrapped around:
     *   7900 -> 100  : delta = -7800, wrapped forward  -> one more round
     *   100  -> 7900 : delta = +7800, wrapped backward -> one round less */
    if (m->rx_count > 0)
    {
        const int32_t delta = (int32_t)ecd - (int32_t)m->ecd;
        if (delta < -(DJI_ECD_RANGE / 2))
        {
            m->total_rounds++;
        }
        else if (delta > (DJI_ECD_RANGE / 2))
        {
            m->total_rounds--;
        }
    }

    m->ecd = ecd;
    /* Assemble as uint16_t first, then reinterpret as signed: 0xFC18 -> -1000.
     * (The old DJI example code kept these as uint16_t, which broke negative values.) */
    m->speed_rpm = (int16_t)(uint16_t)((data[2] << 8) | data[3]);
    m->current_raw = (int16_t)(uint16_t)((data[4] << 8) | data[5]);
    m->temperature = data[6];
    m->rx_count++;
}

/* ------------------------------ public functions ------------------------------ */

DJIMotor_t *DJIMotor_Register(const DJIMotor_Config_t *config)
{
    if (config == NULL || config->bus >= CAN_BUS_NUM)
    {
        return NULL;
    }

    uint32_t feedback_id;
    uint8_t group, slot;
    if (!ComputeIds(config, &feedback_id, &group, &slot))
    {
        return NULL; /* ESC id out of range for this motor type */
    }

    /* Conflict check: on the same bus, two motors must not share a frame position.
     * (Frame position and feedback ID map one-to-one, so this also catches equal feedback IDs,
     * e.g. C620 id 5 and GM6020 id 1, which both use 0x1FF slot 0 and feedback 0x205.) */
    for (uint8_t i = 0; i < motor_count; i++)
    {
        const DJIMotor_t *other = &motor_pool[i];
        if (other->config.bus == config->bus && other->group == group && other->slot == slot)
        {
            return NULL;
        }
    }

    if (motor_count >= DJI_MOTOR_MAX)
    {
        return NULL;
    }

    /* Fill the motor completely before bsp_can can deliver frames to it. */
    DJIMotor_t *motor = &motor_pool[motor_count];
    motor->config = *config;
    motor->output = 0;
    motor->clamp_count = 0;
    motor->group = group;
    motor->slot = slot;

    const CAN_RxConfig_t rx = {
        .bus = config->bus,
        .id = feedback_id,
        .callback = DecodeFeedback,
        .owner = motor, /* comes back as "owner" in DecodeFeedback */
    };
    if (BSP_CAN_RegisterRx(&rx) != CAN_OK)
    {
        return NULL; /* slot not committed: motor_count unchanged */
    }

    frame_used[config->bus][group] = 1;
    motor_count++;
    return motor;
}

void DJIMotor_SetOutput(DJIMotor_t *motor, int32_t output)
{
    if (motor == NULL)
    {
        return;
    }

    const int32_t limit = OutputLimit(motor->config.type);
    if (output > limit)
    {
        output = limit;
        motor->clamp_count++;
    }
    else if (output < -limit)
    {
        output = -limit;
        motor->clamp_count++;
    }
    motor->output = (int16_t)output; /* safe: |output| <= limit <= 25000 */
}

void DJIMotor_GetMeasure(const DJIMotor_t *motor, DJIMotor_Measure_t *out)
{
    if (motor == NULL || out == NULL)
    {
        return;
    }

    /* Critical section: block interrupts so the RX interrupt cannot update measure halfway
     * through the copy. Save and restore PRIMASK instead of blindly re-enabling, so this is
     * also correct if the caller had interrupts disabled already. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = motor->measure;
    __set_PRIMASK(primask);
}

CAN_Status_e DJIMotor_SendAll(void)
{
    uint8_t frame[CAN_BUS_NUM][DJI_FRAME_NUM][8] = {0}; /* unused slots stay 0 = no output */

    for (uint8_t i = 0; i < motor_count; i++)
    {
        const DJIMotor_t *motor = &motor_pool[i];
        uint8_t *bytes = frame[motor->config.bus][motor->group];

        bytes[2 * motor->slot] = (uint8_t)((uint16_t)motor->output >> 8); /* high byte first */
        bytes[2 * motor->slot + 1] = (uint8_t)((uint16_t)motor->output & 0xFF);
    }

    CAN_Status_e result = CAN_OK;
    for (uint8_t bus = 0; bus < CAN_BUS_NUM; bus++)
    {
        for (uint8_t g = 0; g < DJI_FRAME_NUM; g++)
        {
            if (!frame_used[bus][g])
            {
                continue; /* no motor in this frame: don't send an all-zero frame */
            }
            const CAN_Status_e status = BSP_CAN_Transmit((CAN_Bus_e)bus, frame_id[g], frame[bus][g], 8);
            if (status != CAN_OK && result == CAN_OK)
            {
                result = status; /* report the first failure, but still try the other frames */
            }
        }
    }
    return result;
}