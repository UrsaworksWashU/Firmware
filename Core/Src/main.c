/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "dji_motor.h"
#include "pid.h"
#include "dt7.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- Temporary wheel speed loop test. Replace with the chassis application later. ---- */

/* Loop period. HAL_Delay(n) waits at least n+1 ticks, so HAL_Delay(1) gives about 2 ms. */
#define TEST_LOOP_DELAY_MS 1u
#define TEST_LOOP_DT_S     0.002f

/* Watch these in Ozone. Change test_target_rpm (and test_pid.config gains) live. */
CAN_Status_e test_init_status;       /* result of BSP_CAN_Init */
DJIMotor_t *test_motor;              /* NULL means DJIMotor_Register failed */
bool test_pid_init_ok;               /* false means the PID config was rejected */
PID_t test_pid;                      /* rotor speed loop: rpm in, raw current out */
float test_target_rpm = 0.0f;        /* rotor rpm target; try 1000, then negative values */
float test_pid_output;               /* raw current command from the PID */
DJIMotor_Measure_t test_measure;     /* consistent snapshot copied every loop */
CAN_Status_e test_tx_status;         /* result of the latest DJIMotor_SendAll */
const CAN_Stats_t *test_can1_stats;

/* DT7 remote. Watch test_rc while moving sticks and switches. */
UART_Status_e test_rc_init_status;   /* result of DT7_Init */
DT7_Data_t test_rc;                  /* snapshot copied every loop */
bool test_rc_online;                 /* false: no good frame for 100 ms, test_rc is all zeros */
const DT7_Stats_t *test_rc_stats;    /* frames_ok should rise by ~70 per second */
const UART_Stats_t *test_dbus_stats; /* rx_errors rising = wrong UART settings or wiring */
uint8_t test_rc_drives_motor = 0;    /* 1: left stick vertical sets test_target_rpm (PID mode) */
#define TEST_RC_RPM_PER_UNIT 3.0f    /* full stick (660) -> about 2000 rotor rpm */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* A debugger (OpenOCD reset-init) or bootloader may leave the PLL running as SYSCLK,
     which makes HAL_RCC_OscConfig() refuse to reconfigure it. Start from the reset state. */
  HAL_RCC_DeInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_Base_Init(&htim4);
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_Delay(500);
  test_init_status = BSP_CAN_Init();

  /* The M3508 on CAN1 with ESC ID 2: feedback 0x202, control frame 0x200 slot 1. */
  const DJIMotor_Config_t motor_cfg = {
      .type = DJI_MOTOR_M3508,
      .bus = CAN_BUS_1,
      .esc_id = 2,
  };
  test_motor = DJIMotor_Register(&motor_cfg);
  test_can1_stats = BSP_CAN_GetStats(CAN_BUS_1);

  /* Start with P only (ki = 0), tune kp, then add ki. */
  const PID_Config_t pid_cfg = {
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .output_max = 16384.0f, /* M3508 / C620 full scale (about 20 A) */
      .i_out_max = 10000.0f,
      .d_lpf_alpha = 0.0f,
  };
  test_pid_init_ok = PID_Init(&test_pid, &pid_cfg);

  test_rc_init_status = DT7_Init(UART_PORT_DBUS);
  test_rc_stats = DT7_GetStats();
  test_dbus_stats = BSP_UART_GetStats(UART_PORT_DBUS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    test_rc_online = DT7_GetData(&test_rc);
    if (test_rc_drives_motor)
    {
      /* Offline gives left_y = 0, so losing the remote stops the motor. */
      test_target_rpm = (float)test_rc.left_y * TEST_RC_RPM_PER_UNIT;
    }

    if (test_motor != NULL && test_pid_init_ok)
    {
      DJIMotor_GetMeasure(test_motor, &test_measure);
      test_pid_output = PID_Calc(&test_pid, test_target_rpm,
                                 (float)test_measure.speed_rpm, TEST_LOOP_DT_S);
      DJIMotor_SetOutput(test_motor, (int32_t)test_pid_output);
    }
    test_tx_status = DJIMotor_SendAll();
    HAL_Delay(TEST_LOOP_DELAY_MS);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
