/**
 * @file    bsp_time.h
 * @author  eason
 * @brief   Time base for modules, so they never call HAL_GetTick() directly.
 *
 *          Safe to call from interrupts (it only reads a counter). Never busy-wait on it inside an
 *          interrupt: the tick is advanced by SysTick, which has the lowest priority and cannot
 *          run while another interrupt is executing.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/**
 * Milliseconds since boot. Wraps after about 49.7 days.
 * Compare times with unsigned subtraction, (now - then) >= timeout, which is correct across the wrap.
 */
uint32_t BSP_Time_GetMs(void);

#endif /* BSP_TIME_H */