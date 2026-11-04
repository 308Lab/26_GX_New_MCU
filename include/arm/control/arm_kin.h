/**
  ******************************************************************************
  * @file    arm/control/arm_kin.h
  * @brief   机械臂正解/逆解（纯函数）。
  *
  * 关节顺序：J1 底座方位，J2 大臂，J3 小臂，J4 腕部。
  * 位置单位 mm，角度单位 deg。
  ******************************************************************************
  */

#ifndef ARM_CONTROL_ARM_KIN_H
#define ARM_CONTROL_ARM_KIN_H

#include "arm/arm_types.h"

/* 正解：关节角 -> 末端位姿。返回 0 成功。 */
int Arm_Kin_Forward(const float q_deg[4], Arm_Pose_t *pose);

/* 逆解：末端位姿 -> 关节角。不可达返回 -1。 */
int Arm_Kin_Inverse(const Arm_Pose_t *pose, float q_deg[4]);

#endif /* ARM_CONTROL_ARM_KIN_H */
