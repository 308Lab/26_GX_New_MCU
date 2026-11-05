/**
  ******************************************************************************
  * @file    common/watchdog.h
  * @brief   任务级 IWDG 看门狗（审计 H9）。
  *
  * 五个周期任务各自 Watchdog_Kick() 自己的槽位；MainTask02 每拍调用
  * Watchdog_Service()：只有全部槽位都在窗口内更新过才 HAL_IWDG_Refresh()，
  * 有任何任务卡死就不喂狗，让 IWDG 在 ≤2s 内复位（复位前把卡死的任务名记进
  * BKP 备份寄存器，见 common/fault.h）。
  *
  * 编译开关 DEBUG_WITHOUT_IWDG：定义后不启动 IWDG，断点/单步不会被复位。
  * 发布固件不要定义它。
  ******************************************************************************
  */

#ifndef COMMON_WATCHDOG_H
#define COMMON_WATCHDOG_H

#include <stdint.h>

typedef enum
{
    WD_MAIN = 0,   /* MainTask02（5 ms 拍） */
    WD_CAR,        /* CarTask（5 ms 拍） */
    WD_ENC,        /* EncTask（5 ms 拍） */
    WD_ARM,        /* ArmTask（5 ms 拍） */
    WD_OLED,       /* OLEDTask（200 ms 拍，软 I2C 刷屏较慢，窗口 800 ms） */
    WD_NUM
} Wd_Owner_e;

/* 初始化 + 启动 IWDG，并把所有槽位复位为「刚刚喂过」。
   必须在系统真正跑起来之后调用（App_MainTask 校准完成处）：
   Chassis_Calibrate(3000) 是唯一的长阻塞，启动后就没这个问题了。 */
void Watchdog_Start(void);

/* 任务体里每拍调用，标记「我还活着」 */
void Watchdog_Kick(Wd_Owner_e owner);

/* 检查全部槽位，健康则喂狗并返回 1；有任务超时则返回 0（并落盘首次记录） */
uint8_t Watchdog_Service(void);

/* 是否已启动（DEBUG_WITHOUT_IWDG 下恒为 0） */
uint8_t Watchdog_IsRunning(void);

#endif /* COMMON_WATCHDOG_H */
