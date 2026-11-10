/**
 * @file    bsp_buzzer.c
 * @author  eason
 * @brief   
 * @version beta
 * @date    2026-10-20
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}