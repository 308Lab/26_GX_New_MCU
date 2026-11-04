/**
  ******************************************************************************
  * @file    chassis/perception/odom.h
  * @brief   底盘感知层：双编码器里程计 + 陀螺仪航向融合。
  *
  * 航向处理分两件事：
  *   1) 不丢圈 —— 逐帧 Wrap180 取增量，在无界域累积；
  *   2) 不漂移 —— 编码器差速航向与陀螺仪航向互补融合（JY61P 为 6 轴，
  *      Yaw 由角速度积分，长期必然漂移）。
  ******************************************************************************
  */

#ifndef CHASSIS_PERCEPTION_ODOM_H
#define CHASSIS_PERCEPTION_ODOM_H

#include <stdint.h>

typedef struct
{
    float    x_mm, y_mm;          /* 启动点为原点的车体坐标 */
    float    yaw_deg;             /* 融合航向，无界累积 */
    float    yaw_gyro_deg;        /* 纯陀螺仪航向，无界累积 */
    float    yaw_enc_deg;         /* 纯编码器差速航向，无界累积 */
    float    yaw_wz_deg;          /* wz 原始角速度积分航向，诊断用 */
    float    v_mmps, w_degps;     /* 车体线速度 / 角速度（编码器差速） */
    float    yaw_rate_dps;        /* 陀螺仪角度微分得到的角速度（不受打滑影响） */
    float    v_l_mmps, v_r_mmps;  /* 左右轮线速度，供速度环 */
    float    dist_total_mm;       /* 累计里程 */
    float    ds_seg_mm;           /* 本节段位移，MarkSegStart 清零 */
    float    yaw_seg_deg;         /* 本节段转角 */
    uint32_t stamp_ms;
    uint32_t sample_period_ms;    /* 实测采样周期，供 dt 计算 */
} Chassis_Odom_State_t;

void  Chassis_Odom_Init(void);
void  Chassis_Odom_Update(void);
void  Chassis_Odom_GetState(Chassis_Odom_State_t *out);
void  Chassis_Odom_Reset(float x_mm, float y_mm, float yaw_deg);
void  Chassis_Odom_MarkSegStart(void);
float Chassis_Odom_GetYawUnwrapped(void);

#endif /* CHASSIS_PERCEPTION_ODOM_H */
