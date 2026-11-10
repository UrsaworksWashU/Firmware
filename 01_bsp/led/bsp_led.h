/**
 * @file    bsp_led.h
 * @author  eason
 * @brief   bsp for led on C board with PWM for each R/G/B channel
 * @version v1.0 firmware
 * @date    2026-10-17
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_LED_H
#define BSP_LED_H
#include <stdint.h>

extern void aRGB_led_show(uint32_t aRGB);

#endif