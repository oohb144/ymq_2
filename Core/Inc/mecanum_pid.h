#ifndef __MECANUM_PID_H
#define __MECANUM_PID_H

#include "main.h"

/* ============================================================
 * 用户可调参数区 — 4路电机独立 PID 参数
 * ============================================================ */

/* 电机A — KP 降低，加 KD 阻尼消除卡顿 */
#define MOTOR_A_KP      2000
#define MOTOR_A_KI      0
#define MOTOR_A_KD      500

/* 电机B — 加 KD 阻尼 */
#define MOTOR_B_KP      1900
#define MOTOR_B_KI      0
#define MOTOR_B_KD      500

/* 电机C */
#define MOTOR_C_KP      2500
#define MOTOR_C_KI      0
#define MOTOR_C_KD      400

/* 电机D */
#define MOTOR_D_KP      2500
#define MOTOR_D_KI      0
#define MOTOR_D_KD      400

/* PID 缩放系数（避免整数溢出） */
#define MOTOR_PID_SCALE         0.01f

/* 积分限幅（防止积分饱和） */
#define MOTOR_PID_INTEGRAL_MAX  1000

/* PWM 输出限幅（不得超过 MOTOR_PWM_MAX=11999） */
#define MOTOR_PID_OUTPUT_MAX    11999

/* ============================================================
 * 函数声明
 * ============================================================ */
int32_t PID_MotorA(int32_t target, int32_t current);
int32_t PID_MotorB(int32_t target, int32_t current);
int32_t PID_MotorC(int32_t target, int32_t current);
int32_t PID_MotorD(int32_t target, int32_t current);
void    PID_ResetAll(void);

#endif /* __MECANUM_PID_H */
