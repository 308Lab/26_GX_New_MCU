/**
  ******************************************************************************
  * @file    chassis/driver/gyro.h
  * @brief   JY61P 陀螺仪驱动（WIT 协议，USART2，115200 8N1）。
  *
  * 同时解析 0x52 角速度帧与 0x53 角度帧：
  *   yaw 用于位置/航向环，wz 用于内环阻尼。
  * 解析在串口中断中完成，只做整数运算，不调用任何 RTOS API。
  ******************************************************************************
  */

#ifndef CHASSIS_DRIVER_GYRO_H
#define CHASSIS_DRIVER_GYRO_H

#include <stdint.h>

typedef struct
{
    float    yaw_deg;     /* 0x53 解析值，[-180, 180] */
    float    wz_dps;      /* 0x52 解析值，已扣零偏 */
    int16_t  yaw_raw;
    int16_t  wz_raw;
    uint32_t tick_ms;     /* 接收时刻 */
    uint32_t seq;         /* 合法帧序号 */
    uint32_t rx_bytes;    /* USART2 收到的原始字节累计 */
    uint32_t yaw_frames;  /* 0x53 角度帧累计 */
    uint32_t wz_frames;   /* 0x52 角速度帧累计 */
    uint32_t calib_cnt;   /* 零偏标定已采 0x52 帧数 */
    uint32_t calib_target;/* 零偏标定目标帧数 */
    uint32_t err_head;    /* 帧头不同步（事件级计数） */
    uint32_t err_flag;    /* 未定义标志位 */
    uint32_t err_sum;     /* 校验失败 */
    uint8_t  last_bad_flag; /* 最近一次未定义的标志位 */
    uint8_t  has_yaw;
    uint8_t  has_wz;
} Chassis_Gyro_Frame_t;

void     Chassis_Gyro_Init(void);

/* 供串口接收回调喂入单字节；纯计算，可在 ISR 中调用 */
void     Chassis_Gyro_RxByteISR(uint8_t byte);

/* 取最新帧快照；返回 1 表示自上次调用后有新帧 */
uint8_t  Chassis_Gyro_GetFrame(Chassis_Gyro_Frame_t *out);

/* 静止零偏标定：采集 n 帧角速度求均值。标定期间 IsCalibDone()==0 */
void     Chassis_Gyro_StartBiasCalib(uint16_t n_samples);
uint8_t  Chassis_Gyro_IsCalibDone(void);

/* 360° 标定比例系数 */
void     Chassis_Gyro_SetYawScale(float k);

float    Chassis_Gyro_GetYawDeg(void);
float    Chassis_Gyro_GetWzDps(void);
uint8_t  Chassis_Gyro_IsOnline(void);
uint32_t Chassis_Gyro_GetErrorCount(void);

/* WIT 配置命令：FF AA reg lo hi */
void     Chassis_Gyro_SendCmd(uint8_t reg, uint16_t val);

#endif /* CHASSIS_DRIVER_GYRO_H */
