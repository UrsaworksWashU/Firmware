/**
 * @file    robot.c
 * @author  eason
 * @brief   Robot init and the per-cycle order of the applications.
 *
 *          Order each cycle: cmd -> gimbal -> chassis -> send all motor frames.
 *          cmd reads the gimbal feedback published in the previous cycle (one cycle, 2 ms, old).
 *          All motor outputs are set first and sent together at the end, so the four wheels of
 *          one cycle go out in the same CAN frame.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "robot.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "dji_motor.h"
#include "dt7.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"

/* Init results and the latest send result, for watching in Ozone (add "robot_status"). */
typedef struct
{
    CAN_Status_e can_init;
    UART_Status_e rc_init;
    CAN_Status_e last_send;
    uint32_t task_count;
} Robot_Status_t;

static Robot_Status_t robot_status;

void Robot_Init(void)
{
    /* bsp and device modules first, then the applications that use them. */
    robot_status.can_init = BSP_CAN_Init();
    robot_status.rc_init = DT7_Init(UART_PORT_DBUS);

    Gimbal_Init();
    Chassis_Init();
    RobotCmd_Init();
}

void Robot_Task(float dt_s)
{
    RobotCmd_Task(dt_s);
    Gimbal_Task(dt_s);
    Chassis_Task(dt_s);

    robot_status.last_send = DJIMotor_SendAll();
    robot_status.task_count++;
}