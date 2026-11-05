/**
  ******************************************************************************
  * @file    common/fault.h
  * @brief   故障记录：把复位前的故障码/出错任务名写进 BKP 备份寄存器。
  *
  * 审计 H8/H9：本板没有 LED（main.h 里只有 Laser/OLED/键扫描/电机/编码器），
  * 所以「点亮 LED 报警」不可实现；改用 STM32 的 42 字节备份寄存器当黑匣子：
  *   BKP->DR1  故障码（0 = 无记录）
  *   BKP->DR2  ／ BKP->DR3  出错任务名前 4 个字符（每寄存器 2 字符）
  *   BKP->DR4  出错时刻 HAL_GetTick() 的低 16 位
  *   BKP->DR5  「连续致命复位」计数，系统真正跑起来（看门狗启动）时清零
  * 备份域由 VBAT/内部电源维持，复位（含 IWDG 复位）后仍可读；开机时
  * Fault_Init() 读出并清零，再由 OLED 首页第 7 行显示，人眼就能看出上次
  * 是什么原因死的。用 SWD 调试时直接看 BKP->DR1~DR5 也行。
  ******************************************************************************
  */

#ifndef COMMON_FAULT_H
#define COMMON_FAULT_H

#include <stdint.h>

/* 故障码（写进 BKP->DR1，必须非 0 才算有记录） */
#define FAULT_CODE_NONE             (0x0000U)
#define FAULT_CODE_STACK_OVERFLOW   (0x0001U)
#define FAULT_CODE_MALLOC_FAILED    (0x0002U)
#define FAULT_CODE_HARDFAULT        (0x0003U)
#define FAULT_CODE_MEMMANAGE_FAULT  (0x0004U)
#define FAULT_CODE_BUSFAULT         (0x0005U)
#define FAULT_CODE_USAGEFAULT       (0x0006U)
#define FAULT_CODE_ERROR_HANDLER    (0x0007U)
#define FAULT_CODE_WDG_STALE        (0x0008U)

/* 开机最早调用：打开备份域访问，把上次的记录读进静态变量后清零 */
void Fault_Init(void);

/* 记录一次故障。task_name 可为 NULL，只取前 4 个字符 */
void Fault_Record(uint16_t code, const char *task_name);

/* Fault_Init() 读到的记录（Fault_Init 之前恒为 0/NULL） */
uint16_t    Fault_GetLastCode(void);
const char *Fault_GetLastTaskName(void);

/* 记录 + 计数 + NVIC_SystemReset()；连续 3 次同一路径起不来时改为停机，
   避免在初始化阶段死循环复位。HardFault/Error_Handler 用这个。 */
void Fault_Fatal(uint16_t code);

/* 关中断停机（看门狗若已启动，会在 ≤2s 内复位，记录已落盘） */
void Fault_Halt(void);

#endif /* COMMON_FAULT_H */
