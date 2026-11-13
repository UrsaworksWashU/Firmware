/**
 * @file    robot.h
 * @author  eason
 * @brief   The whole robot: initializes bsp and every application, then runs them in order.
 *          main.c only calls Robot_Init once and Robot_Task every period.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ROBOT_H
#define ROBOT_H

/** Call once, after the CubeMX MX_*_Init functions. */
void Robot_Init(void);

/** Call every ROBOT_TASK_PERIOD_S. dt_s is passed down to every application. */
void Robot_Task(float dt_s);

#endif /* ROBOT_H */