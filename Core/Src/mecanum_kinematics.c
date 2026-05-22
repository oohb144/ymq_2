#include "mecanum_kinematics.h"
#include <math.h>

/* 运行时参数（由 Kinematics_Init 计算） */
static float s_ticks_per_meter  = 0.0f;
static float s_wheel_track_cali = 0.0f;

/* 正运动学内部状态 */
static int32_t s_current_count[4] = {0};
static int32_t s_wheel_mult[4]    = {0};

/* ============================================================
 * 初始化
 * ============================================================ */
void Kinematics_Init(void)
{
    /* 每米对应的编码器脉冲数 */
    s_ticks_per_meter = MECANUM_ENCODER_RESOLUTION
                        / (MECANUM_WHEEL_DIAMETER * MECANUM_PI * MECANUM_LINEAR_CORRECTION);

    /* 校正后的等效半轮距（用于旋转分量计算） */
    s_wheel_track_cali = MECANUM_WHEEL_TRACK / MECANUM_ANGULAR_CORRECTION;

    /* 清零里程计内部状态 */
    for (int i = 0; i < 4; i++) {
        s_current_count[i] = 0;
        s_wheel_mult[i]    = 0;
    }
}

/* ============================================================
 * 逆运动学：目标速度 → 各轮目标编码器增量
 *
 * 麦克纳姆轮运动学方程（X型布局，轮序：左前/右前/左后/右后）：
 *   v0(左前) = -vy + vx - w * L
 *   v1(右前) =  vy + vx + w * L
 *   v2(左后) =  vy + vx - w * L
 *   v3(右后) = -vy + vx + w * L
 * ============================================================ */
void Kinematics_Inverse(int16_t *input, int16_t *output)
{
    float vx  = (float)input[0] / 1000.0f;   /* m/s */
    float vy  = (float)input[1] / 1000.0f;
    float w   = (float)input[2] / 1000.0f;   /* rad/s */

    float wheel_vel[4];
    wheel_vel[0] = -vy + vx - s_wheel_track_cali * w;
    wheel_vel[1] =  vy + vx + s_wheel_track_cali * w;
    wheel_vel[2] =  vy + vx - s_wheel_track_cali * w;
    wheel_vel[3] = -vy + vx + s_wheel_track_cali * w;

    /* 转换为每个 PID 周期的期望脉冲数 */
    for (int i = 0; i < 4; i++) {
        output[i] = (int16_t)(wheel_vel[i] * s_ticks_per_meter / MECANUM_PID_RATE);
    }
}

/* ============================================================
 * 正运动学：编码器累计值 → 里程计
 * ============================================================ */
void Kinematics_Forward(int16_t *input, int16_t *output)
{
    /* 参考例程中各轮方向符号（根据实际安装方向调整）：
     * 左前(A)取反、右前(B)不变、左后(C)取反、右后(D)不变 */
    int16_t recv[4];
    recv[0] = -input[0];
    recv[1] =  input[1];
    recv[2] = -input[2];
    recv[3] =  input[3];

    /* 处理编码器溢出（16位有符号计数器翻转） */
    for (int i = 0; i < 4; i++) {
        if (recv[i] < ENCODER_LOW_WRAP && s_current_count[i] > ENCODER_HIGH_WRAP)
            s_wheel_mult[i]++;
        else if (recv[i] > ENCODER_HIGH_WRAP && s_current_count[i] < ENCODER_LOW_WRAP)
            s_wheel_mult[i]--;
        else
            s_wheel_mult[i] = 0;
    }

    /* 编码器脉冲 → 位移（m） */
    double delta[4];
    for (int i = 0; i < 4; i++) {
        delta[i] = (double)(recv[i]
                   + s_wheel_mult[i] * (ENCODER_MAX_VAL - ENCODER_MIN_VAL)
                   - s_current_count[i]) / s_ticks_per_meter;
        s_current_count[i] = recv[i];
    }

    /* 机器人坐标系下的位移分量 */
    double dx   = (delta[3] - delta[2]) / 2.0;
    double dy   = (delta[2] - delta[0]) / 2.0;
    double dyaw = (delta[2] + delta[1]) / (2.0 * s_wheel_track_cali);

    /* 旋转到里程计坐标系 */
    double dx_odom = cos(dyaw) * dx - sin(dyaw) * dy;
    double dy_odom = -sin(dyaw) * dx - cos(dyaw) * dy;

    double yaw_now = (double)output[2] / 1000.0;
    output[0] += (int16_t)((cos(yaw_now) * dx_odom - sin(yaw_now) * dy_odom) * 1000.0);
    output[1] += (int16_t)((sin(yaw_now) * dx_odom + cos(yaw_now) * dy_odom) * 1000.0);
    output[2] += (int16_t)(dyaw * 1000.0);

    /* Yaw 角限制在 [-π, π] */
    if (output[2] >  (int16_t)(MECANUM_PI * 1000.0f)) output[2] -= (int16_t)(2.0f * MECANUM_PI * 1000.0f);
    if (output[2] < -(int16_t)(MECANUM_PI * 1000.0f)) output[2] += (int16_t)(2.0f * MECANUM_PI * 1000.0f);

    /* 输出速度分量 */
    output[3] = (int16_t)(dx   * 1000.0);
    output[4] = (int16_t)(-dy  * 1000.0);
    output[5] = (int16_t)(dyaw * 1000.0);
}
