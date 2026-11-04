/**
  ******************************************************************************
  * @file    chassis/control/ctrl.h
  * @brief   底盘控制层：角度环 -> 速度环串级、减速规划、到点判定、反向脉冲制动。
  *
  * 本头文件仅供底盘规划层使用。app 层不包含它——app 只通过 chassis.h 的
  * 节拍入口驱动控制层。
  *
  * 解耦要点：指令只携带“相对转角”，绝对航向、段起点打点、无界域累积
  * 全部由控制层内部维护，规划层无需知道里程计/陀螺仪的存在。
  ******************************************************************************
  */

#ifndef CHASSIS_CONTROL_CTRL_H
#define CHASSIS_CONTROL_CTRL_H

#include <stdint.h>

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_STRAIGHT,    /* 航向环(外) + 速度环(内) */
    CHASSIS_MODE_SPIN,        /* 原地旋转 */
    CHASSIS_MODE_ARC,         /* 定半径过弯 */
    CHASSIS_MODE_SPEED_RAW    /* 纯速度给定，调试用 */
} Chassis_Mode_e;

typedef struct
{
    Chassis_Mode_e mode;
    float    v_mmps;             /* 目标线速度，符号 = 前后 */
    float    heading_delta_deg;  /* 相对转角，逆时针（左转）为正 */
    float    radius_mm;          /* ARC 半径，正 = 左转 */
    float    dist_mm;            /* 目标位移；<= 0 表示不定长 */
    float    end_speed_mmps;     /* 段末目标速度；0 = 停车，非 0 = 平滑衔接下一段 */
    uint8_t  brake_pulse;        /* 到点后是否反向脉冲制动 */
    uint8_t  seg_last;           /* 是否路径末段 */
    uint16_t seq;                /* 指令序号 */
} Chassis_Cmd_t;

typedef struct
{
    uint16_t       seq;
    Chassis_Mode_e mode;
    uint8_t        finished;
    uint8_t        fault;        /* 陀螺仪掉线 / 超时 */
    float          v_meas_mmps;
    float          w_meas_degps;
    float          w_cmd_degps;   /* 航向环输出，诊断用 */
    float          heading_meas_deg;
    float          dist_done_mm;
    int16_t        pwm_l;
    int16_t        pwm_r;
} Chassis_Status_t;

typedef enum
{
    CHASSIS_WAIT_DONE = 0,
    CHASSIS_WAIT_TIMEOUT,
    CHASSIS_WAIT_FAULT
} Chassis_Wait_e;

void Chassis_Ctrl_Init(void);

/* 由底盘子系统内部在 osKernelInitialize() 之后调用 */
void Chassis_Ctrl_InitRtos(void);

/* 路径开始：把计划航向锚定到当前实测航向 */
void Chassis_Ctrl_BeginPath(void);

/* 控制环单拍，由 Chassis_Control_Update() 转发 */
void Chassis_Ctrl_Update(void);

/* 下发一段。内部自动累加绝对航向、MarkSegStart、入队、序号管理 */
void Chassis_Ctrl_Submit(const Chassis_Cmd_t *cmd, uint8_t reset_integral);

/* 阻塞等待本段完成（内部等待队列，句柄不外露） */
Chassis_Wait_e Chassis_Ctrl_WaitDone(Chassis_Status_t *st, uint32_t timeout_ms);

void Chassis_Ctrl_GetStatus(Chassis_Status_t *out);
void Chassis_Ctrl_Stop(uint8_t brake);

/* 调试接管：置 1 后控制环不再输出，电机由调试接口直接控制 */
void Chassis_Ctrl_SetDebugOverride(uint8_t on);

/* 调试速度环：复用正式左右轮 PID，给定左右目标轮速（mm/s）。
   持续运行直到 StopSpeed。用于 S9 速度环整定。 */
void Chassis_Ctrl_DebugRunSpeed(float v_l_mmps, float v_r_mmps);
void Chassis_Ctrl_DebugStopSpeed(void);

/* 调试直行：航向环(外) + 速度环(内)，走完指定距离后自动制动。
   用于 S10 直行验证。非阻塞，结果通过状态取。 */
void Chassis_Ctrl_DebugRunStraight(float dist_mm, float speed_mmps);
void Chassis_Ctrl_DebugStopStraight(void);

/* 调试原地旋转：转到指定相对角度（正 = 左转）。
   复用正式旋转环 PID。非阻塞。 */
void Chassis_Ctrl_DebugRunSpin(float angle_deg);

/* 调试圆弧：半径 radius_mm（正=左转），转过 angle_deg，线速度 speed_mmps。
   当前为纯运动学前馈，用于采集基线误差。非阻塞。 */
void Chassis_Ctrl_DebugRunArc(float radius_mm, float angle_deg, float speed_mmps);

#endif /* CHASSIS_CONTROL_CTRL_H */
