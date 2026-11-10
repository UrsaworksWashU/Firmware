/**
 * @file    bsp_buzzer.h
 * @author  eason
 * @brief   
 * @version beta
 * @date    2026-10-20
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include <stdint.h>

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif