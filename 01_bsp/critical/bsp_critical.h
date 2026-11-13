/**
 * @file    bsp_critical.h
 * @author  eason
 * @brief   Short critical sections that modules can use without including main.h or CMSIS.
 *
 *          Usage:
 *              const uint32_t state = BSP_EnterCritical();
 *              ... a few copies of data shared with an interrupt ...
 *              BSP_ExitCritical(state);
 *
 *          Rules:
 *          - Keep the section to a few copies: every interrupt (CAN, UART, SysTick) waits meanwhile.
 *          - No return / break / goto between Enter and Exit: Exit would be skipped
 *            and interrupts would stay off forever.
 *          - Nesting is safe: Exit restores the state saved by its own Enter, it does not blindly
 *            re-enable interrupts (that is why this saves PRIMASK instead of calling __enable_irq()).
 *          - DMA keeps running inside the section; only interrupts are held back.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H

#include <stdint.h>

/* The one place that pulls in CMSIS for interrupt masking. A different board only changes this include. */
#include "stm32f4xx.h"

/** Disable interrupts and return the previous PRIMASK (0 = interrupts were enabled). */
static inline uint32_t BSP_EnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/** Restore the PRIMASK saved by the matching BSP_EnterCritical. */
static inline void BSP_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#endif /* BSP_CRITICAL_H */
