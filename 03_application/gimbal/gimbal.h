/**
 * @file    gimbal.h
 * @author  eason
 * @brief   Gimbal application. For now only the yaw GM6020, held at zero force:
 *          it reports where the gimbal points relative to the chassis (topic_gimbal_feedback).
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef GIMBAL_H
#define GIMBAL_H

void Gimbal_Init(void);
void Gimbal_Task(float dt_s);

#endif /* GIMBAL_H */