/**
  ******************************************************************************
  * @file    chassis/driver/encoder.h
  * @brief   底盘编码器驱动：TIM1(左) / TIM8(右) 硬件编码器接口。
  *
  * ReadDelta 为破坏性读，全工程仅允许感知层调用一次。
  ******************************************************************************
  */

#ifndef CHASSIS_DRIVER_ENCODER_H
#define CHASSIS_DRIVER_ENCODER_H

#include <stdint.h>

typedef enum
{
    CHASSIS_ENC_LEFT = 0,
    CHASSIS_ENC_RIGHT = 1,
    CHASSIS_ENC_NUM
} Chassis_Enc_Id_e;

void Chassis_Encoder_Init(void);

/* 自上次调用以来的脉冲增量，带符号；内部用 int16 差值处理 16 位回绕 */
int16_t Chassis_Encoder_ReadDelta(Chassis_Enc_Id_e id);

/* 软件累加的 32 位脉冲总数 */
int32_t Chassis_Encoder_GetTotal(Chassis_Enc_Id_e id);

void Chassis_Encoder_ResetTotal(Chassis_Enc_Id_e id);

#endif /* CHASSIS_DRIVER_ENCODER_H */
