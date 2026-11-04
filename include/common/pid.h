/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器。
  *
  * 特性：输出限幅、积分钳位、输出饱和时冻结积分、积分分离、误差死区、
  *       微分对测量值求取并低通。dt 必须由调用方传入实测周期。
  ******************************************************************************
  */

#ifndef COMMON_PID_H
#define COMMON_PID_H

#include <stdint.h>

typedef struct
{
    /* 参数 */
    float kp;
    float ki;
    float kd;

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 积分项限幅（抗饱和） */
    float i_min;
    float i_max;

    /* 微分低通系数 0~1，1 表示不滤波 */
    float d_lpf_alpha;

    /* 误差死区：|err| 小于该值时 P/D 照算，但积分不更新 */
    float deadband;

    /* 积分分离阈值：|err| 大于该值时暂停积分；<=0 关闭 */
    float i_sep_thresh;

    /* 运行时状态 */
    float integral;
    float prev_meas;
    float d_filt;
    uint8_t initialized;
} Pid_t;

void  Pid_Init(Pid_t *pid, float kp, float ki, float out_min, float out_max);
void  Pid_SetGains(Pid_t *pid, float kp, float ki, float kd);
void  Pid_SetLimits(Pid_t *pid, float out_min, float out_max,
                    float i_min, float i_max);
void  Pid_SetFilter(Pid_t *pid, float d_lpf_alpha, float deadband,
                    float i_sep_thresh);
void  Pid_Reset(Pid_t *pid);
float Pid_Update(Pid_t *pid, float setpoint, float measure, float dt_s);

#endif /* COMMON_PID_H */
