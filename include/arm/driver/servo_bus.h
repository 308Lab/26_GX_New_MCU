/**
  ******************************************************************************
  * @file    arm/driver/servo_bus.h
  * @brief   众灵 ZP45D / ZP10D 总线舵机（ASCII 协议），UART4 半双工。
  *          协议与 ZX361S 同族，见 3683724937总线舵机通信协议.pdf。
  *
  * 本层只处理 PWM 与 ASCII 帧，不负责角度换算。
  ******************************************************************************
  */

#ifndef ARM_DRIVER_SERVO_BUS_H
#define ARM_DRIVER_SERVO_BUS_H

#include <stdint.h>

void Arm_Servo_Init(void);
/* 在 osKernelInitialize() 之后调用：创建半双工总线的发送互斥量（审计 H10）。 */
void Arm_Servo_InitRtos(void);
void Arm_Servo_Update(void);

/* 单舵机 / 多舵机同步位置写。time_ms 为舵机内部运动时间。 */
int Arm_Servo_SetPwm(uint8_t id, uint16_t pwm, uint16_t time_ms);
int Arm_Servo_SetPwmSync(const uint8_t *ids, const uint16_t *pwm,
                         uint8_t n, uint16_t time_ms);

/* 位置回读：请求异步，结果由 GetPositionPwm 取。 */
void Arm_Servo_RequestPosition(uint8_t id);
int  Arm_Servo_GetPositionPwm(uint8_t id, uint16_t *pwm);
void Arm_Servo_ClearPositionCache(void);

/* 最近一次位置回读的时刻（HAL_GetTick），0 表示从未回读。
   同步帧既无长度也无 ACK，上层用它确认回读真的刷新过（审计 H11）。 */
uint32_t Arm_Servo_GetRxStampMs(uint8_t id);

void Arm_Servo_Torque(uint8_t id, uint8_t on);
uint8_t Arm_Servo_GetTorque(uint8_t id);

/* 广播释放/恢复所有舵机扭矩： #255PULK! / #255PULR! */
void Arm_Servo_TorqueBroadcast(uint8_t on);

/* 把当前位置设为开机启动位置（PCSD）。 */
int Arm_Servo_SetStartPosition(uint8_t id);

void Arm_Servo_Stop(uint8_t id);
void Arm_Servo_StopAll(void);

/* ID 配置：一次只接一个出厂 ID 0 的舵机。改 ID 后建议断电重启。 */
int  Arm_Servo_SetId(uint8_t old_id, uint8_t new_id);
void Arm_Servo_RequestId(uint8_t id);
int  Arm_Servo_GetLastId(uint8_t *id);

/* ID 操作结果：0=无/等待，1=成功，2=失败（50 ms 内无对应应答） */
uint8_t Arm_Servo_GetIdResult(void);

/* 广播查询：发送 #255PID!，从应答 #IDP! 中解析 ID。
   一次只接一个舵机时使用。 */
void Arm_Servo_RequestBroadcastQuery(void);

/* 地址扫描：逐 ID 发送 #IDPID!，收集有应答的 ID。 */
void    Arm_Servo_ScanStart(uint8_t first, uint8_t last);
uint8_t Arm_Servo_ScanIsActive(void);
uint8_t Arm_Servo_ScanIsDone(void);
uint8_t Arm_Servo_ScanCount(void);
uint8_t Arm_Servo_ScanId(uint8_t index);

uint8_t  Arm_Servo_IsOnline(uint8_t id);
uint32_t Arm_Servo_GetErrorCount(uint8_t id);

#endif /* ARM_DRIVER_SERVO_BUS_H */
