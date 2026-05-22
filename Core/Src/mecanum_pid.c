#include "mecanum_pid.h"

typedef struct {
    int32_t pwm_out;
    int32_t bias_last;
    int32_t bias_integral;
} PidState_t;

static PidState_t pid_a = {0, 0, 0};
static PidState_t pid_b = {0, 0, 0};
static PidState_t pid_c = {0, 0, 0};
static PidState_t pid_d = {0, 0, 0};

static int32_t pid_calc(PidState_t *s, int32_t target, int32_t current,
                        int32_t kp, int32_t ki, int32_t kd)
{
    int32_t bias = target - current;

    s->bias_integral += bias;
    if (s->bias_integral >  MOTOR_PID_INTEGRAL_MAX) s->bias_integral =  MOTOR_PID_INTEGRAL_MAX;
    if (s->bias_integral < -MOTOR_PID_INTEGRAL_MAX) s->bias_integral = -MOTOR_PID_INTEGRAL_MAX;

    s->pwm_out += (int32_t)(kp * bias                      * MOTOR_PID_SCALE
                          + kd * (bias - s->bias_last)     * MOTOR_PID_SCALE
                          + ki * s->bias_integral          * MOTOR_PID_SCALE);

    s->bias_last = bias;

    if (s->pwm_out >  MOTOR_PID_OUTPUT_MAX) s->pwm_out =  MOTOR_PID_OUTPUT_MAX;
    if (s->pwm_out < -MOTOR_PID_OUTPUT_MAX) s->pwm_out = -MOTOR_PID_OUTPUT_MAX;

    return s->pwm_out;
}

int32_t PID_MotorA(int32_t target, int32_t current)
{ return pid_calc(&pid_a, target, current, MOTOR_A_KP, MOTOR_A_KI, MOTOR_A_KD); }

int32_t PID_MotorB(int32_t target, int32_t current)
{ return pid_calc(&pid_b, target, current, MOTOR_B_KP, MOTOR_B_KI, MOTOR_B_KD); }

int32_t PID_MotorC(int32_t target, int32_t current)
{ return pid_calc(&pid_c, target, current, MOTOR_C_KP, MOTOR_C_KI, MOTOR_C_KD); }

int32_t PID_MotorD(int32_t target, int32_t current)
{ return pid_calc(&pid_d, target, current, MOTOR_D_KP, MOTOR_D_KI, MOTOR_D_KD); }

void PID_ResetAll(void)
{
    pid_a = (PidState_t){0, 0, 0};
    pid_b = (PidState_t){0, 0, 0};
    pid_c = (PidState_t){0, 0, 0};
    pid_d = (PidState_t){0, 0, 0};
}
