/**
  ******************************************************************************
  * @file    pid.c
  ******************************************************************************
  */

#include "common/pid.h"
#include "common/math_util.h"

#define PID_DT_MIN_S   (0.0001f)

void Pid_Init(Pid_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min = out_min * 0.5f;
    pid->i_max = out_max * 0.5f;

    pid->d_lpf_alpha = 1.0f;
    pid->deadband = 0.0f;
    pid->i_sep_thresh = 0.0f;

    Pid_Reset(pid);
}

void Pid_SetGains(Pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void Pid_SetLimits(Pid_t *pid, float out_min, float out_max,
                   float i_min, float i_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min = i_min;
    pid->i_max = i_max;
    pid->integral = Math_Clamp(pid->integral, i_min, i_max);
}

void Pid_SetFilter(Pid_t *pid, float d_lpf_alpha, float deadband,
                   float i_sep_thresh)
{
    pid->d_lpf_alpha = Math_Clamp(d_lpf_alpha, 0.0f, 1.0f);
    pid->deadband = deadband;
    pid->i_sep_thresh = i_sep_thresh;
}

void Pid_Reset(Pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
    pid->initialized = 0;
}

float Pid_Update(Pid_t *pid, float setpoint, float measure, float dt_s)
{
    float err;
    float p;
    float d_raw;
    float d;
    float i_new;
    float out;
    uint8_t allow_integral;

    if (dt_s < PID_DT_MIN_S)
    {
        dt_s = PID_DT_MIN_S;
    }

    err = setpoint - measure;

    /* 微分对测量值求取，避免设定值阶跃冲击 */
    if (!pid->initialized)
    {
        pid->prev_meas = measure;
        pid->initialized = 1;
        d_raw = 0.0f;
    }
    else
    {
        d_raw = -(measure - pid->prev_meas) / dt_s;
    }
    pid->prev_meas = measure;

    pid->d_filt = pid->d_lpf_alpha * pid->d_filt
                + (1.0f - pid->d_lpf_alpha) * d_raw;
    d = pid->kd * pid->d_filt;

    p = pid->kp * err;

    /* 积分条件：误差在死区外，且未触发积分分离 */
    allow_integral = 1;
    if (pid->deadband > 0.0f
        && err < pid->deadband && err > -pid->deadband)
    {
        allow_integral = 0;
    }
    if (pid->i_sep_thresh > 0.0f
        && (err > pid->i_sep_thresh || err < -pid->i_sep_thresh))
    {
        allow_integral = 0;
    }

    i_new = pid->integral;
    if (allow_integral)
    {
        i_new += pid->ki * err * dt_s;
        i_new = Math_Clamp(i_new, pid->i_min, pid->i_max);
    }

    out = p + i_new + d;

    /* 输出饱和且误差指向饱和方向时冻结积分 */
    if ((out > pid->out_max && err > 0.0f)
        || (out < pid->out_min && err < 0.0f))
    {
        /* 不采纳本拍积分 */
    }
    else
    {
        pid->integral = i_new;
    }

    return Math_Clamp(out, pid->out_min, pid->out_max);
}
