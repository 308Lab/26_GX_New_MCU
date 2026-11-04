/**
  ******************************************************************************
  * @file    keypad.h
  * @brief   3x3 矩阵键盘（调试输入），严格无阻塞扫描。
  *
  * 布局（俯视）：
  *   第一行 7 8 9
  *   第二行 4 5 6
  *   第三行 1 2 3
  *
  * 行：PE10(1) / PB2(2) / PE7(3)，推挽输出
  * 列：PE13(1) / PE14(2) / PE15(3)，上拉输入
  *
  * 扫描采用分相状态机：每次 Keypad_Scan() 只推进一个相位并立即返回，
  * 不使用任何 HAL_Delay / while 等待 / NOP 稳定延时。
  * 两拍之间的节拍间隔天然满足行列电平稳定时间。
  ******************************************************************************
  */

#ifndef COMMON_KEYPAD_H
#define COMMON_KEYPAD_H

#include <stdint.h>

#define KEYPAD_KEY_NONE         (0U)

/* MainTask02 节拍 */
#define KEYPAD_SCAN_PERIOD_MS   (5U)

/* 一轮完整扫描的拍数：DRIVE0 -> READ0 -> READ1 -> READ2 */
#define KEYPAD_ROUND_TICKS      (4U)

/* 连续 N 轮原始键值一致才确认 */
#define KEYPAD_DEBOUNCE_ROUNDS  (2U)

void    Keypad_Init(void);

/* 周期调用（建议 5ms），单拍内只做一个相位，立即返回 */
void    Keypad_Scan(void);

/* 取一次“新按下”事件，取后清除；无事件返回 KEYPAD_KEY_NONE */
uint8_t Keypad_GetEvent(void);

/* 当前稳定按下的键值字符，未按下返回 KEYPAD_KEY_NONE */
uint8_t Keypad_GetState(void);

#endif /* COMMON_KEYPAD_H */
