/**
  ******************************************************************************
  * @file    arm/control/arm_motion.h
  * @brief   关节空间 PTP：多轴时间对齐、下发、回读到位判定。
  *
  * 一次 Start 只接受一条指令；由 Arm_Motion_Update() 非阻塞推进。
  ******************************************************************************
  */

#ifndef ARM_CONTROL_ARM_MOTION_H
#define ARM_CONTROL_ARM_MOTION_H

#include "arm/arm_types.h"

int  Arm_Motion_Init(void);

/* 启动一次 PTP。返回 ARM_OK / ARM_BUSY / ARM_ERR_LIMIT / ARM_ERR_COMM / ARM_ERR_PARAM。 */
Arm_Status_e Arm_Motion_StartJoint(const float q_target[4], float min_time_s);

void         Arm_Motion_Update(void);
uint8_t      Arm_Motion_IsBusy(void);
uint8_t      Arm_Motion_IsDone(void);
uint8_t      Arm_Motion_HasFault(void);
Arm_Status_e Arm_Motion_GetError(void);

void Arm_Motion_Abort(void);

#endif /* ARM_CONTROL_ARM_MOTION_H */
