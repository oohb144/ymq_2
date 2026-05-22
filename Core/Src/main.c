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
#include "memorymap.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mecanum_motor.h"
#include "mecanum_pid.h"
#include "mecanum_kinematics.h"
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
/* 编码器累计值（各轮，有符号） */
static int16_t g_encoder_acc[4]    = {0};
/* 每个控制周期编码器增量 */
static int16_t g_encoder_delta[4]  = {0};
/* 逆运动学输出：各轮目标增量 */
static int16_t g_encoder_target[4] = {0};
/* 里程计：{x(mm), y(mm), yaw(mrad), vx, vy, w} */
static int16_t g_odom[6]           = {0};
/* 目标速度：{vx(mm/s), vy(mm/s), w(mrad/s)} */
int16_t g_target_speed[3]          = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void MecanumControl_50Hz(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Kinematics_Init();
g_target_speed[0] = -500;
  /* 启动编码器定时器，并立即将计数器置为中值，避免第一帧读出 -32768 导致猛冲 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 32768);
  __HAL_TIM_SET_COUNTER(&htim4, 32768);
  __HAL_TIM_SET_COUNTER(&htim5, 32768);
  __HAL_TIM_SET_COUNTER(&htim8, 32768);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_ctrl_tick = 0;   /* 50Hz 控制节拍 */
  uint32_t last_led_tick  = 0;   /* LED 闪烁节拍 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 50Hz 麦轮控制循环（每 20ms 执行一次） */
    if (now - last_ctrl_tick >= 20) {
      last_ctrl_tick = now;
      MecanumControl_50Hz();
    }

    /* LED 500ms 翻转闪烁（低电平亮，PC13）
     * 作为看门狗指示：LED 持续闪烁说明主循环未卡死 */
    if (now - last_led_tick >= 500) {
      last_led_tick = now;
      HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ============================================================
 * 50Hz 麦克纳姆轮控制主函数
 * 流程：读编码器增量 → 正运动学 → 逆运动学 → PID → 设置电机
 * ============================================================ */
static void MecanumControl_50Hz(void)
{
    /* ============================================================
     * 裸机转向测试模式（TEST_OPEN_LOOP = 1）
     * 绕过 PID，直接给定固定 PWM，用于确认每路电机方向是否正确
     * 确认方向无误后将此宏改为 0，切回闭环 PID 模式
     * ============================================================ */
#define TEST_OPEN_LOOP  0

/* 裸机测试 PWM 值（范围 0 ~ 11999，建议从小值开始，如 2000~4000） */
#define TEST_PWM_VALUE  3000

#if TEST_OPEN_LOOP

    /* 直接输出固定 PWM，正值=正转，负值=反转
     * 麦轮向前运动时四路电机应全部朝同一方向转动（由安装方向决定）
     * 若某路方向反了，修改 mecanum_motor.h 中对应的 MOTOR_x_POLARITY */
    Motor_A_SetSpeed( TEST_PWM_VALUE);
    Motor_B_SetSpeed( TEST_PWM_VALUE);
    Motor_C_SetSpeed( TEST_PWM_VALUE);
    Motor_D_SetSpeed( TEST_PWM_VALUE);

#else  /* 闭环 PID 模式 */

    /* 1. 读取编码器增量（计数器中值法，避免溢出）
     *    TIM3=电机A, TIM4=电机B, TIM5=电机C, TIM8=电机D */
    #define ENC_MID     32768u
    /* 单帧最大合理增量：500mm/s对应约26脉冲/帧，限幅设为150（约6倍余量）
     * 超过此值视为干扰帧，保持上一帧的值不更新 */
    #define ENC_MAX_DELTA  150

    int16_t raw_delta[4];
    raw_delta[0] =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim3) - ENC_MID);  /* A 正向 */
    raw_delta[1] =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim4) - ENC_MID);  /* B */
    raw_delta[2] = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim5) - ENC_MID);  /* C 取反 */
    raw_delta[3] =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim8) - ENC_MID);  /* D */

    __HAL_TIM_SET_COUNTER(&htim3, ENC_MID);
    __HAL_TIM_SET_COUNTER(&htim4, ENC_MID);
    __HAL_TIM_SET_COUNTER(&htim5, ENC_MID);
    __HAL_TIM_SET_COUNTER(&htim8, ENC_MID);

    /* 限幅过滤：异常跳变帧用上一帧值替代，避免 PID 因干扰脉冲失控 */
    for (int i = 0; i < 4; i++) {
        if (raw_delta[i] >  ENC_MAX_DELTA) raw_delta[i] =  ENC_MAX_DELTA;
        if (raw_delta[i] < -ENC_MAX_DELTA) raw_delta[i] = -ENC_MAX_DELTA;
        g_encoder_delta[i] = raw_delta[i];
    }

    /* 2. 累计编码器值（用于正运动学里程计） */
    for (int i = 0; i < 4; i++) {
        g_encoder_acc[i] += g_encoder_delta[i];
    }

    /* 3. 正运动学：编码器累计 → 里程计 */
    Kinematics_Forward(g_encoder_acc, g_odom);

    /* 4. 逆运动学：目标速度 → 各轮目标增量 */
    Kinematics_Inverse(g_target_speed, g_encoder_target);

    /* 5. PID 速度控制 */
    int32_t pwm_a = PID_MotorA(g_encoder_target[0], g_encoder_delta[0]);
    int32_t pwm_b = PID_MotorB(g_encoder_target[1], g_encoder_delta[1]);
    int32_t pwm_c = PID_MotorC(g_encoder_target[2], g_encoder_delta[2]);
    int32_t pwm_d = PID_MotorD(g_encoder_target[3], g_encoder_delta[3]);

    /* 6. 输出到电机（符号参考原例程，调试时若方向错误修改此处正负号） */
    Motor_A_SetSpeed( pwm_a);
    Motor_B_SetSpeed( pwm_b);
    Motor_C_SetSpeed( pwm_c);
    Motor_D_SetSpeed( pwm_d);

#endif /* TEST_OPEN_LOOP */
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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
