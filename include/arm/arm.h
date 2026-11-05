/**
  ******************************************************************************
  * @file    arm/arm.h
  * @brief   机械臂子系统唯一对外入口。app 层只包含本文件。
  *
  * 正式动作全部委托 planning 层：arm.c -> planning -> control -> perception/driver。
  ******************************************************************************
  */

#ifndef ARM_ARM_H
#define ARM_ARM_H

#include <stdint.h>

#include "arm/arm_types.h"

/* 初始化全部驱动、感知、控制与动作层。须在 RTOS 启动前调用。 */
Arm_Status_e Arm_Init(void);

/* 节拍入口：由 App_MainTask 每 5 ms 调用一次，非阻塞。 */
void Arm_Update(void);

/* 安全停止：停止运动、关闭激光；夹爪保持当前状态。 */
void Arm_Stop(void);

/* 状态查询 */
uint8_t      Arm_IsBusy(void);
Arm_Status_e Arm_GetLastError(void);

/* 正式动作：启动后由 Arm_Update 推进；重复启动返回 ARM_BUSY。 */
Arm_Status_e Arm_Home(uint16_t timeout_ms);
Arm_Status_e Arm_Stow(uint16_t timeout_ms);
Arm_Status_e Arm_MoveJoint(const float q_deg[4], uint16_t timeout_ms);
Arm_Status_e Arm_MovePose(const Arm_Pose_t *pose, uint16_t timeout_ms);

/* 状态查询（正式模式伺服/上报用） */
Arm_Status_e Arm_GetTcpPose(Arm_Pose_t *out);
Arm_Status_e Arm_GetJoints(float q_deg[4]);

/* 工具系平面微调：dx 左、dy 上，单位 mm（协议 §2） */
Arm_Status_e Arm_NudgeTool(int16_t dx_mm, int16_t dy_mm, uint16_t timeout_ms);

/* 沿工具法向（指向目标）平移 dist_mm，正=接近 */
Arm_Status_e Arm_ApproachTool(int16_t dist_mm, uint16_t timeout_ms);

/* 三舵机 PWM 位姿（ID1/ID2/ID3 = J2/J3/J4），直接下发同步帧 */
Arm_Status_e Arm_MovePwm(const uint16_t pwm[3], uint16_t time_ms);

/* 只设置腕部 J4 的 PWM（大臂/小臂保持不动） */
Arm_Status_e Arm_SetWristPwm(uint16_t pwm, uint16_t time_ms);

/* J1 底座相对旋转（度），正 = 左转 */
Arm_Status_e Arm_MoveBaseRelative(float delta_deg);
Arm_Status_e Arm_PickAndPlace(const Arm_Pose_t *pick,
                              const Arm_Pose_t *place,
                              float approach_mm,
                              uint16_t timeout_ms);
Arm_Status_e Arm_Aim(const Arm_Pose_t *aim_pose, uint16_t timeout_ms);

/* 夹爪与激光 */
Arm_Status_e Arm_Grasp(void);   /* 夹取 105° */
Arm_Status_e Arm_Close(void);   /* 闭合 135° */
Arm_Status_e Arm_Hold(void);
Arm_Status_e Arm_Release(void);
Arm_Status_e Arm_Laser(uint8_t on);

#endif /* ARM_ARM_H */
