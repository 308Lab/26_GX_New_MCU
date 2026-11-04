/**
  ******************************************************************************
  * @file    arm/control/arm_limit.h
  * @brief   关节软限位与工作空间检查（纯函数）。
  ******************************************************************************
  */

#ifndef ARM_CONTROL_ARM_LIMIT_H
#define ARM_CONTROL_ARM_LIMIT_H

#include "arm/arm_types.h"

/* 返回 0 表示全部在限位内，-1 表示超限。 */
int Arm_Limit_CheckJoints(const float q_deg[4]);

/* 检查末端位置是否在工作空间内，并验证逆解关节是否超限。 */
int Arm_Limit_CheckPose(const Arm_Pose_t *pose);

#endif /* ARM_CONTROL_ARM_LIMIT_H */
