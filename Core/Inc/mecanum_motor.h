#ifndef __MECANUM_MOTOR_H
#define __MECANUM_MOTOR_H

#include "main.h"

/* ============================================================
 * 用户可调参数区 — 根据实际硬件修改以下宏
 * ============================================================ */

/* PWM 最大占空比（对应 TIM2 ARR=11999，即 100% 占空比）
 * 若需限制最大转速，可将此值调小，例如 8000 = 约 67% */
#define MOTOR_PWM_MAX       11999

/* 电机方向极性（+1 或 -1）
 * 若某路电机实际转向与期望相反，将对应值改为 -1 */
#define MOTOR_A_POLARITY    (-1)//zs
#define MOTOR_B_POLARITY    (-1)//yx
#define MOTOR_C_POLARITY    (-1)//zx
#define MOTOR_D_POLARITY    (-1)//ys

/* ============================================================
 * 内部定义（无需修改）
 * ============================================================ */

/* 电机通道枚举 */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_C = 2,
    MOTOR_D = 3
} MotorChannel_t;

/* 函数声明 */
void Motor_Init(void);
void Motor_A_SetSpeed(int32_t speed);
void Motor_B_SetSpeed(int32_t speed);
void Motor_C_SetSpeed(int32_t speed);
void Motor_D_SetSpeed(int32_t speed);
void Motor_SetAllStop(void);

#endif /* __MECANUM_MOTOR_H */
