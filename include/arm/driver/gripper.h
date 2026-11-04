/**
  ******************************************************************************
  * @file    arm/driver/gripper.h
  * @brief   夹爪 PWM 驱动，TIM4 CH4（PD15，50 Hz）。
  ******************************************************************************
  */

#ifndef ARM_DRIVER_GRIPPER_H
#define ARM_DRIVER_GRIPPER_H

#include <stdint.h>

void Arm_Gripper_Init(void);
void Arm_Gripper_SetPulseUs(uint16_t pulse_us);
void Arm_Gripper_Open(void);
void Arm_Gripper_Close(void);
void Arm_Gripper_Grasp(void);
uint16_t Arm_Gripper_GetPulseUs(void);

#endif /* ARM_DRIVER_GRIPPER_H */
