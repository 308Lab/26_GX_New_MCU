/**
  ******************************************************************************
  * @file    app/app_debug.h
  * @brief   OLED 调试视图。不依赖 stdio，避免 512B 任务栈溢出。
  ******************************************************************************
  */

#ifndef APP_APP_DEBUG_H
#define APP_APP_DEBUG_H

#include <stdint.h>

/* 当前调试步骤。按键效果随步骤变化，每一步在自己的 case 中实现 */
typedef enum
{
    DEBUG_STEP_S1 = 0,   /* 基础运行：OLED + 键盘识别 */
    DEBUG_STEP_S3,       /* 陀螺仪链路 */
    DEBUG_STEP_S4,       /* 电机方向与极性 */
    DEBUG_STEP_S5,       /* 编码器方向与线数 */
    DEBUG_STEP_S6,       /* PWM-速度特性 */
    DEBUG_STEP_S7,       /* 里程计标定 */
    DEBUG_STEP_S8,       /* 陀螺仪方向/比例/漂移 */
    DEBUG_STEP_S9,       /* 速度环整定 */
    DEBUG_STEP_S10,      /* 直行 */
    DEBUG_STEP_S11,      /* 原地旋转 */
    DEBUG_STEP_S12,      /* 圆弧 */
    DEBUG_STEP_S13,      /* 停车制动 */
    DEBUG_STEP_S14,      /* 整条路径 */
    DEBUG_STEP_S15,      /* 机械臂 J1 步进 */
    DEBUG_STEP_S16,      /* 机械臂 J2~J4 舵机 */
    DEBUG_STEP_S17,      /* 机械臂夹爪/激光 */
    DEBUG_STEP_S18,      /* 机械臂 PTP/动作 */
    DEBUG_STEP_S19,      /* 机械臂舵机 ID 配置 */
    DEBUG_STEP_S20,      /* 机械臂单舵机校准 */
    DEBUG_STEP_S21       /* J3 换装：扫描/改 ID/回读/点动/写启动位 */
} App_Debug_Step_e;

/* 当前调试阶段：按测试内容裁剪入口与显示，不做运行时切页。
   测底盘其他项目时，把 APP_DEBUG_FIXED_STEP 改为对应 DEBUG_STEP_Sx 后重新编译。 */
#define APP_STAGE_CHASSIS   (1U)
#define APP_STAGE_ARM       (2U)
#define APP_STAGE_MISSION   (3U)

#ifndef APP_STAGE
#define APP_STAGE           APP_STAGE_MISSION
#endif

#if (APP_STAGE == APP_STAGE_CHASSIS)
#ifndef APP_DEBUG_FIXED_STEP
#define APP_DEBUG_FIXED_STEP  DEBUG_STEP_S14
#endif
#elif (APP_STAGE == APP_STAGE_ARM)
#define APP_DEBUG_FIXED_STEP  DEBUG_STEP_S20
#else
#define APP_DEBUG_FIXED_STEP  DEBUG_STEP_S14
#endif

/* 定点格式化，buf 建议至少 16 字节 */
void App_Dbg_FmtFloat(char *buf, float v, uint8_t decimals);
void App_Dbg_FmtInt(char *buf, int32_t v);

/* 采集底盘只读快照并刷新 OLED */
void App_Debug_Refresh(void);

/* 点动测试时长（ms） */
#define APP_DEBUG_JOG_MS    (500U)

/* S5 脉冲-距离标定时使用的 PWM */
#define APP_DEBUG_PULSE_PWM (250)

/* 调试阶段模拟任务执行时长（毫秒）；接入真实任务层后移除 */
#define APP_DEBUG_TASK_MS   (500U)

/* 整条路径串行执行时，每个任务点的停留时间 */
#define APP_DEBUG_CHAIN_DWELL_MS (2000U)

void App_Debug_SetStep(App_Debug_Step_e step);

/* 上电陀螺零偏校准结果：0=进行中，1=OK，2=失败。仅用于页头显示。 */
void App_Debug_SetGyroCalib(uint8_t state);
App_Debug_Step_e App_Debug_GetStep(void);

/* 每 5ms 调用一次：处理点动到期等非阻塞调试动作 */
void App_Debug_Update(void);

/* 每 5ms 调用一次：处理路径层事件（扫描/到点/完成） */
void App_Debug_HandlePathEvents(void);

/* 演示流程（无视觉）公共入口。正式模式扫码失败或模拟按键时接管。 */
void    App_Debug_StartDemo(void);        /* 从起点开始完整演示 */
void    App_Debug_StartToA(void);         /* 纯路径：起点→A，停车后结束 */
void    App_Debug_StartDemoFromA(void);   /* A 点扫码失败：J1 已右转，从回正继续 */
void    App_Debug_StartDemoFromC(void);   /* 仅跑 C：从通用展开开始，跑完收起即停 */
void    App_Debug_StopDemo(void);
uint8_t App_Debug_IsDemoActive(void);

/* 按键分发。硬约束：本函数内禁止任何阻塞调用
   （不得使用 osDelay / HAL_Delay / Chassis_Calibrate / Chassis_Path_Run /
     HAL_UART_Transmit 等）。动作只能通过非阻塞接口表达。 */
void App_Debug_HandleKey(uint8_t key);

uint8_t App_Debug_GetLastKey(void);

#endif /* APP_APP_DEBUG_H */
