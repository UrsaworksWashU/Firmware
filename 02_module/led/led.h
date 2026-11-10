/**
 * @file    led.h
 * @author  eason
 * @brief   
 * @version beta
 * @date    2026-10-17
 * @copyright Copyright (c) 2026
 *
 */

#ifndef LED_H
#define LED_H

#define RGB_FLOW_COLOR_CHANGE_TIME  100
#define RGB_FLOW_COLOR_LENGHTH   3

/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
extern void led_rainbow_flow();

#endif //LED_H
