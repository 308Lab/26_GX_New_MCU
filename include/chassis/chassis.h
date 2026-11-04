/**
  ******************************************************************************
  * @file    chassis/chassis.h
  * @brief   底盘子系统唯一对外入口。
  *
  * app 层只包含本文件。内部 driver / perception / control / planning 四层
  * 的接口不出现在这里，也不允许 app 层直接调用。
  ******************************************************************************
  */

#ifndef CHASSIS_CHASSIS_H
#define CHASSIS_CHASSIS_H

#include <stdint.h>
#include "chassis/planning/path.h"

/* 初始化全部子层。app 层不接触任何 HAL 句柄 */
void Chassis_Init(void);

/* 在 osKernelInitialize() 之后创建静态队列 */
void Chassis_InitRtos(void);

/* 陀螺仪零偏标定：阻塞直到完成或超时。必须在车体静止时调用。
   返回 0 = 成功，-1 = 超时 */
int Chassis_Calibrate(uint32_t timeout_ms);

/* 非阻塞版本：启动后由 Chassis_CalibratePoll() 查询完成状态。
   用于路径层在任务点停车后重新标定零偏。 */
void    Chassis_CalibrateStart(void);
uint8_t Chassis_CalibratePoll(void);

/* 三个节拍入口：app 层只负责按周期调用，不参与控制逻辑 */
void Chassis_Sense_Update(void);      /* EncTask，5 ms */
void Chassis_Control_Update(void);    /* CarTask，5 ms */

/* 路径接口由 chassis/planning/path.h 提供（已在上方 include） */

/* 急停 */
void Chassis_Control_Stop(uint8_t brake);

/* 把当前航向设为新的零点（非阻塞，供调试按键使用） */
void Chassis_ResetHeading(void);

/* 调试：直接给定左右轮 PWM，期间控制环让出电机。用于 S4 电机方向测试 */
void Chassis_Debug_SetMotor(int16_t left_pwm, int16_t right_pwm);
void Chassis_Debug_ReleaseMotor(void);

/* 调试：双轮以固定 PWM 前进，直到平均累计脉冲达到目标后短路制动。
   用于 S5 脉冲-距离标定。全程非阻塞，由 Chassis_Debug_Update 推进。 */
void    Chassis_Debug_RunPulses(int32_t target_pulses, int16_t pwm);
void    Chassis_Debug_StopPulses(void);
void    Chassis_Debug_Update(void);
void    Chassis_Debug_ResetPulses(void);
void    Chassis_Debug_GetPulses(int32_t *left, int32_t *right);

/* 调试：左右独立目标轮速闭环（mm/s），复用正式 PID 参数。
   用于 S9 速度环整定，持续运行直到 StopSpeed。 */
void    Chassis_Debug_RunSpeed(float left_mmps, float right_mmps);
void    Chassis_Debug_StopSpeed(void);

/* 调试直行：航向环 + 速度环串级，走完 dist_mm 后自动制动 */
void    Chassis_Debug_RunStraight(float dist_mm, float speed_mmps);
void    Chassis_Debug_StopStraight(void);

/* 调试原地旋转到相对角度（正 = 左转） */
void    Chassis_Debug_RunSpin(float angle_deg);

/* 调试圆弧：半径正=左转，转过指定角度 */
void    Chassis_Debug_RunArc(float radius_mm, float angle_deg, float speed_mmps);

/* 只读调试快照，不暴露任何内部结构或全局变量 */
typedef struct
{
    float    heading_deg;      /* 编码器/陀螺仪融合航向 */
    float    yaw_gyro_deg;     /* 纯陀螺仪累积航向，S3 标定用 */
    float    yaw_enc_deg;      /* 纯编码器差速航向，诊断用 */
    float    yaw_wz_deg;       /* wz 原始角速度积分航向，诊断用 */
    float    wz_dps;           /* 陀螺仪角速度 */
    float    dist_mm;
    float    v_mmps;
    float    v_l_mmps;
    float    v_r_mmps;
    float    w_degps;
    float    w_cmd_degps;      /* 航向环输出，S10 诊断 */
    int16_t  pwm_l;
    int16_t  pwm_r;
    uint8_t  gyro_online;
    uint8_t  calib_done;
    uint8_t  gyro_bad_flag;
    uint32_t gyro_seq;
    uint32_t gyro_rx_bytes;
    uint32_t gyro_yaw_frames;
    uint32_t gyro_wz_frames;
    uint32_t gyro_calib_cnt;
    uint32_t gyro_calib_target;
    uint32_t gyro_err;
    uint32_t gyro_err_head;
    uint32_t gyro_err_flag;
    uint32_t gyro_err_sum;
} Chassis_Debug_View_t;

void Chassis_Debug_GetView(Chassis_Debug_View_t *out);

#endif /* CHASSIS_CHASSIS_H */
