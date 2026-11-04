/**
  ******************************************************************************
  * @file    arm/arm_debug.h
  * @brief   机械臂调试接口。只允许 app_debug.c 包含，不进入正式业务路径。
  ******************************************************************************
  */

#ifndef ARM_ARM_DEBUG_H
#define ARM_ARM_DEBUG_H

#include <stdint.h>

#include "arm/arm_types.h"

typedef struct
{
    float        q_deg[ARM_AXIS_NUM];
    Arm_Status_e status;
    uint8_t      axis_online[ARM_AXIS_NUM];
    uint8_t      step_online;
    uint8_t      laser_on;
    uint8_t      laser_available;
    uint8_t      gripper_closed;
    uint8_t      step_fw;
    uint8_t      step_hw;
    uint8_t      step_err_kind;
    uint8_t      step_last_ack;
    uint8_t      servo_last_id;
    uint8_t      servo_id_valid;
    uint8_t      servo_id_result;   /* 0=等待/无，1=成功，2=失败 */
    uint16_t     servo_pwm[3];      /* J2/J3/J4 最近回读 PWM */
    uint8_t      servo_torque[3];   /* J2/J3/J4 扭矩状态 1=保持 */
    uint32_t     err[ARM_AXIS_NUM];
    uint8_t      action;            /* Arm_Action_e */
    uint8_t      motion_busy;
} Arm_Debug_View_t;

void         Arm_Debug_GetView(Arm_Debug_View_t *out);

/* J1 步进相对点动（正 = 配置的正方向） */
Arm_Status_e Arm_Debug_StepJog(float delta_deg);
Arm_Status_e Arm_Debug_StepStop(void);
Arm_Status_e Arm_Debug_StepEnable(uint8_t on);
void         Arm_Debug_StepRead(void);

/* J2/J3/J4 舵机点动。axis = ARM_AXIS_J2..J4 */
Arm_Status_e Arm_Debug_ServoJog(uint8_t axis, float delta_deg);
void         Arm_Debug_ServoRead(void);

Arm_Status_e Arm_Debug_Gripper(uint8_t close);
Arm_Status_e Arm_Debug_GripperGrasp(void);
Arm_Status_e Arm_Debug_MovePwm(const uint16_t pwm[3], uint16_t time_ms);
Arm_Status_e Arm_Debug_Laser(uint8_t on);

/* 舵机 ID 配置（一次只接一个默认 ID 0 的舵机） */
Arm_Status_e Arm_Debug_ServoSetId(uint8_t old_id, uint8_t new_id);
void         Arm_Debug_ServoRequestId(uint8_t id);
void         Arm_Debug_ServoReadTarget(uint8_t id);
int          Arm_Debug_ServoGetPosition(uint8_t id, uint16_t *pwm);
void         Arm_Debug_ServoClearCache(void);
void         Arm_Debug_ServoTorque(uint8_t id, uint8_t on);
void         Arm_Debug_ServoTorqueBroadcast(uint8_t on);
uint8_t      Arm_Debug_ServoGetTorque(uint8_t id);
void         Arm_Debug_ServoSetStart(uint8_t id);

/* 合并动作：ID1 -> 1823，ID2 -> 1771，同步帧 */
void         Arm_Debug_ServoPreset12(void);

/* 广播查询 ID（#255PRAD!，一次只接一个舵机） */
void Arm_Debug_ServoQueryBroadcast(void);

/* 地址扫描（逐 ID 查询） */
void    Arm_Debug_ServoScanStart(void);
uint8_t Arm_Debug_ServoScanIsActive(void);
uint8_t Arm_Debug_ServoScanIsDone(void);
uint8_t Arm_Debug_ServoScanCount(void);
uint8_t Arm_Debug_ServoScanId(uint8_t index);

/* 联合动作调试：经 control/planning，用于 PTP 与 IK 验证 */
Arm_Status_e Arm_Debug_Home(void);
Arm_Status_e Arm_Debug_Stow(void);
Arm_Status_e Arm_Debug_MoveJoint(const float q_deg[4]);
Arm_Status_e Arm_Debug_MovePose(const Arm_Pose_t *pose);
Arm_Status_e Arm_Debug_MoveTestJoint(void);
Arm_Status_e Arm_Debug_MoveTestPose(void);

#endif /* ARM_ARM_DEBUG_H */
