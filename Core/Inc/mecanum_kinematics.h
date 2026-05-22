#ifndef __MECANUM_KINEMATICS_H
#define __MECANUM_KINEMATICS_H

#include "main.h"

/* ============================================================
 * 用户可调参数区 — 根据实际机器人尺寸和电机参数修改
 * ============================================================ */

/* 等效轮距（m）：lx + ly，其中 lx = 前后轴距/2，ly = 左右轮距/2
 * 影响旋转运动精度，直线运动不受影响
 * 当前值：0.375m（用户实测） */
#define MECANUM_WHEEL_TRACK         0.375f

/* 轮径（m）：麦克纳姆轮外径
 * 当前值：0.100m（用户实测 100mm） */
#define MECANUM_WHEEL_DIAMETER      0.100f

/* 编码器分辨率（脉冲/转）：电机输出轴每圈编码器脉冲数
 * 注意：使用 TI12 双边沿模式时已包含 ×4，直接填实际脉冲数
 * 当前值：827.2（用户提供） */
#define MECANUM_ENCODER_RESOLUTION  827.2f

/* PID 控制频率（Hz）：与主循环控制频率一致，影响速度目标值换算 */
#define MECANUM_PID_RATE            50

/* 线速度校正系数（1.0 = 不校正，调大则里程计线速度偏小） */
#define MECANUM_LINEAR_CORRECTION   1.0f

/* 角速度校正系数（1.0 = 不校正，调大则里程计旋转偏小） */
#define MECANUM_ANGULAR_CORRECTION  1.0f

/* 速度限幅（m/s × 1000，即 mm/s） */
#define MECANUM_LINEAR_SPEED_LIMIT  5000
#define MECANUM_ANGULAR_SPEED_LIMIT 5000

/* ============================================================
 * 内部常量（无需修改）
 * ============================================================ */
#define MECANUM_PI          3.14159265f
#define ENCODER_MAX_VAL     32767
#define ENCODER_MIN_VAL     (-32768)
#define ENCODER_LOW_WRAP    ((int32_t)((ENCODER_MAX_VAL - ENCODER_MIN_VAL) * 0.3f + ENCODER_MIN_VAL))
#define ENCODER_HIGH_WRAP   ((int32_t)((ENCODER_MAX_VAL - ENCODER_MIN_VAL) * 0.7f + ENCODER_MIN_VAL))

/* ============================================================
 * 函数声明
 * ============================================================ */

/* 初始化运动学参数，必须在使用前调用 */
void Kinematics_Init(void);

/* 逆运动学：目标速度 → 各轮目标编码器增量
 * input[3]:  {vx(mm/s), vy(mm/s), w(mrad/s)}  均 ×1000 整数表示
 * output[4]: 每 PID 周期各轮期望编码器脉冲数 */
void Kinematics_Inverse(int16_t *input, int16_t *output);

/* 正运动学：编码器累计值 → 里程计
 * input[4]:  各轮编码器累计脉冲（有符号）
 * output[6]: {x(mm), y(mm), yaw(mrad), vx, vy, w} */
void Kinematics_Forward(int16_t *input, int16_t *output);

#endif /* __MECANUM_KINEMATICS_H */
