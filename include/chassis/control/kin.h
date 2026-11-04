/**
  ******************************************************************************
  * @file    chassis/control/kin.h
  * @brief   差速运动学正逆解与单位换算。
  *
  * 正解: v = (vL + vR) / 2,  w = (vR - vL) / B
  * 逆解: vL = v - w*B/2,     vR = v + w*B/2
  * B = CHASSIS_TRACK_MM，w 单位为 deg/s。
  ******************************************************************************
  */

#ifndef CHASSIS_CONTROL_KIN_H
#define CHASSIS_CONTROL_KIN_H

#include <stdint.h>

void  Chassis_Kin_Forward(float vL_mmps, float vR_mmps,
                          float *v_mmps, float *w_degps);

void  Chassis_Kin_Inverse(float v_mmps, float w_degps,
                          float *vL_mmps, float *vR_mmps);

/* 原地旋转：vL = -vR */
void  Chassis_Kin_Spin(float w_degps, float *vL_mmps, float *vR_mmps);

/* 定半径圆弧：w = v/R，radius_mm 为带符号量（正=左转） */
void  Chassis_Kin_Arc(float v_mmps, float radius_mm,
                      float *vL_mmps, float *vR_mmps);

/* 由弧长与转角反推半径 */
float Chassis_Kin_RadiusFrom(float arc_len_mm, float angle_deg);

float Chassis_Kin_MmToPulse(float mm);
float Chassis_Kin_PulseToMm(int32_t pulse);

#endif /* CHASSIS_CONTROL_KIN_H */
