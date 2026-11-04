/**
  ******************************************************************************
  * @file    arm/driver/step_uart.h
  * @brief   ZDT X42S 闭环步进（Emm 固件）串口驱动，USART1 115200 8N1。
  *
  * 本层只做帧编解码、发送、接收与超时；角度换算与状态管理由上层完成。
  ******************************************************************************
  */

#ifndef ARM_DRIVER_STEP_UART_H
#define ARM_DRIVER_STEP_UART_H

#include <stdint.h>

void Arm_Step_Init(void);
void Arm_Step_Update(void);

/* 命令接口：返回 0 成功，-1 忙/参数错误。 */
int Arm_Step_Enable(uint8_t on);
int Arm_Step_StopNow(void);

/* 相对位置，单位为驱动器脉冲；speed_rpm 由上层按同步时间计算。
   角度换算由上层完成。 */
int Arm_Step_MoveRelativePulses(int32_t pulses, uint16_t speed_rpm);

int Arm_Step_SyncTrigger(void);

/* 回读请求与结果。请求为异步，结果在下一次 Update 后有效。 */
void Arm_Step_RequestPosition(void);
int  Arm_Step_GetPositionDeg(float *deg);
void Arm_Step_RequestVersion(void);
int  Arm_Step_GetVersion(uint8_t *fw, uint8_t *hw);

uint8_t  Arm_Step_IsOnline(void);
uint32_t Arm_Step_GetErrorCount(void);

/* 诊断：最后一次错误类型（1=发送失败，2=超时，3=校验，4=未知功能码，5=ACK 状态异常）
   与最后一次 ACK 状态字节。 */
uint8_t Arm_Step_GetLastErrorKind(void);
uint8_t Arm_Step_GetLastAck(void);

#endif /* ARM_DRIVER_STEP_UART_H */
