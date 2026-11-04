/**
  ******************************************************************************
  * @file    arm/arm_types.h
  * @brief   机械臂对外中立类型。只依赖标准头，不包含任何内部层。
  ******************************************************************************
  */

#ifndef ARM_ARM_TYPES_H
#define ARM_ARM_TYPES_H

#include <stdint.h>

typedef enum
{
    ARM_OK = 0,
    ARM_BUSY,
    ARM_ERR_PARAM,
    ARM_ERR_LIMIT,
    ARM_ERR_UNREACHABLE,
    ARM_ERR_TIMEOUT,
    ARM_ERR_COMM,
    ARM_ERR_ESTOP
} Arm_Status_e;

typedef enum
{
    ARM_AXIS_J1 = 0,   /* 底座旋转（步进） */
    ARM_AXIS_J2,       /* 大臂 */
    ARM_AXIS_J3,       /* 小臂 */
    ARM_AXIS_J4,       /* 腕部 */
    ARM_AXIS_NUM
} Arm_Axis_e;

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
    float tool_deg;
} Arm_Pose_t;

typedef enum
{
    ARM_GRIPPER_OPEN = 0,
    ARM_GRIPPER_CLOSE,
    ARM_GRIPPER_GRASP,
    ARM_GRIPPER_HOLD
} Arm_Gripper_State_e;

#endif /* ARM_ARM_TYPES_H */
