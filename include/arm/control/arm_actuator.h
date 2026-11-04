/**
  ******************************************************************************
  * @file    arm/control/arm_actuator.h
  * @brief   夹爪与激光的语义封装。control 层唯一调用 gripper/laser 驱动的模块。
  ******************************************************************************
  */

#ifndef ARM_CONTROL_ARM_ACTUATOR_H
#define ARM_CONTROL_ARM_ACTUATOR_H

#include "arm/arm_types.h"

void Arm_Actuator_Init(void);

int Arm_Actuator_Gripper(Arm_Gripper_State_e state);
int Arm_Actuator_Laser(uint8_t on);

Arm_Gripper_State_e Arm_Actuator_GetGripper(void);
uint8_t Arm_Actuator_LaserIsOn(void);

#endif /* ARM_CONTROL_ARM_ACTUATOR_H */
