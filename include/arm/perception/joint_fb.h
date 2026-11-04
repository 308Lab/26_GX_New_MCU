/**
  ******************************************************************************
  * @file    arm/perception/joint_fb.h
  * @brief   关节反馈：回读调度、原始量换算、在线状态。
  *
  * 依赖 driver，不依赖 control/planning。
  ******************************************************************************
  */

#ifndef ARM_PERCEPTION_JOINT_FB_H
#define ARM_PERCEPTION_JOINT_FB_H

#include <stdint.h>

#define ARM_FB_AXIS_NUM   (4U)

void Arm_Fb_Init(void);
void Arm_Fb_Update(void);

/* 触发一轮快速回读（调试用） */
void Arm_Fb_RequestAll(void);

/* 取得全部关节角（deg，已扣零位与方向）。返回 1 表示至少一个轴有效。 */
int Arm_Fb_GetJoints(float q_deg[ARM_FB_AXIS_NUM]);

/* 同时返回有效位掩码：bit0=J1, bit1=J2, bit2=J3, bit3=J4。 */
int Arm_Fb_GetJointsMask(float q_deg[ARM_FB_AXIS_NUM], uint8_t *mask);

int      Arm_Fb_IsOnline(uint8_t axis);
uint32_t Arm_Fb_GetErrorCount(uint8_t axis);
int      Arm_Fb_GetServoPwm(uint8_t axis, uint16_t *pwm);
int      Arm_Fb_GetStepPosition(float *deg);

/* J1 软件位置积分：每次成功下发相对角度后调用，便于回 0°。 */
void     Arm_Fb_AddJ1Delta(float delta_deg);

#endif /* ARM_PERCEPTION_JOINT_FB_H */
