#include "mecanum_motor.h"
#include "tim.h"

/* ============================================================
 * 硬件映射（与 CubeMX 配置一致，无需修改）
 *
 * 电机A: PWM=TIM2_CH1(PA15)  方向=AIN1(PA4)/AIN2(PC4)
 * 电机B: PWM=TIM2_CH2(PB3)   方向=BIN1(PB7)/BIN2(PB8)
 * 电机C: PWM=TIM2_CH3(PA2)   方向=CIN1(PC10)/CIN2(PC11)
 * 电机D: PWM=TIM2_CH4(PA3)   方向=DIN1(PE14)/DIN2(PE15)
 * ============================================================ */

/* 启动所有 PWM 通道 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    Motor_SetAllStop();
}

/* 内部辅助：设置单路 PWM 占空比 */
static void set_pwm(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t duty)
{
    if (duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
    __HAL_TIM_SET_COMPARE(htim, channel, duty);
}

/* ============================================================
 * 电机A — TIM2_CH1，方向：AIN1(PA4) / AIN2(PC4)
 * speed > 0: 正转  speed < 0: 反转  speed = 0: 停止
 * ============================================================ */
void Motor_A_SetSpeed(int32_t speed)
{
    speed *= MOTOR_A_POLARITY;

    if (speed > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_1, (uint32_t)speed);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        set_pwm(&htim2, TIM_CHANNEL_1, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_1, 0);
    }
}

/* ============================================================
 * 电机B — TIM2_CH2，方向：BIN1(PB7) / BIN2(PB8)
 * ============================================================ */
void Motor_B_SetSpeed(int32_t speed)
{
    speed *= MOTOR_B_POLARITY;

    if (speed > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_2, (uint32_t)speed);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        set_pwm(&htim2, TIM_CHANNEL_2, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_2, 0);
    }
}

/* ============================================================
 * 电机C — TIM2_CH3，方向：CIN1(PC10) / CIN2(PC11)
 * ============================================================ */
void Motor_C_SetSpeed(int32_t speed)
{
    speed *= MOTOR_C_POLARITY;

    if (speed > 0) {
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_3, (uint32_t)speed);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
        set_pwm(&htim2, TIM_CHANNEL_3, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_3, 0);
    }
}

/* ============================================================
 * 电机D — TIM2_CH4，方向：DIN1(PE14) / DIN2(PE15)
 * ============================================================ */
void Motor_D_SetSpeed(int32_t speed)
{
    speed *= MOTOR_D_POLARITY;

    if (speed > 0) {
        HAL_GPIO_WritePin(DIN__GPIO_Port, DIN__Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_4, (uint32_t)speed);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(DIN__GPIO_Port, DIN__Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
        set_pwm(&htim2, TIM_CHANNEL_4, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(DIN__GPIO_Port, DIN__Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
        set_pwm(&htim2, TIM_CHANNEL_4, 0);
    }
}

/* 全部停止 */
void Motor_SetAllStop(void)
{
    Motor_A_SetSpeed(0);
    Motor_B_SetSpeed(0);
    Motor_C_SetSpeed(0);
    Motor_D_SetSpeed(0);
}
