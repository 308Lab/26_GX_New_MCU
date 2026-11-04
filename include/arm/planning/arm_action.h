/**
  ******************************************************************************
  * @file    arm/planning/arm_action.h
  * @brief   非阻塞动作序列。只调用 control 层，不直接访问驱动/感知。
  ******************************************************************************
  */

#ifndef ARM_PLANNING_ARM_ACTION_H
#define ARM_PLANNING_ARM_ACTION_H

#include <stdint.h>

#include "arm/arm_types.h"

typedef enum
{
    ARM_ACT_NONE = 0,
    ARM_ACT_HOME,
    ARM_ACT_STOW,
    ARM_ACT_JOINT,
    ARM_ACT_POSE,
    ARM_ACT_PICK_PLACE,
    ARM_ACT_AIM,
    ARM_ACT_HOLD,
    ARM_ACT_RELEASE
} Arm_Action_e;

void Arm_Act_Init(void);

Arm_Status_e Arm_Act_StartHome(uint16_t timeout_ms);
Arm_Status_e Arm_Act_StartStow(uint16_t timeout_ms);
Arm_Status_e Arm_Act_StartJoint(const float q_deg[4], uint16_t timeout_ms);
Arm_Status_e Arm_Act_StartPose(const Arm_Pose_t *pose, uint16_t timeout_ms);
Arm_Status_e Arm_Act_StartPickPlace(const Arm_Pose_t *pick,
                                    const Arm_Pose_t *place,
                                    float approach_mm,
                                    uint16_t timeout_ms);
Arm_Status_e Arm_Act_StartAim(const Arm_Pose_t *aim_pose, uint16_t timeout_ms);
Arm_Status_e Arm_Act_Hold(void);
Arm_Status_e Arm_Act_Release(void);

void         Arm_Act_Update(void);
uint8_t      Arm_Act_IsBusy(void);
Arm_Action_e Arm_Act_Current(void);
Arm_Status_e Arm_Act_GetError(void);
void         Arm_Act_Abort(void);

#endif /* ARM_PLANNING_ARM_ACTION_H */
