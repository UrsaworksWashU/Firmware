/**
 * @file    bsp_time.c
 * @author  eason
 * @brief   Time base for modules. See bsp_time.h.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_time.h"
#include "stm32f4xx_hal.h" /* HAL_GetTick: uwTick, advanced by SysTick every 1 ms */

uint32_t BSP_Time_GetMs(void)
{
    return HAL_GetTick();
}