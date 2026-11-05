/**
  ******************************************************************************
  * @file    app/app_tasks.c
  * @brief   任务体：只负责节拍与编排，不含控制逻辑。
  *
  * osDelayUntil 的参数是绝对 tick；目标已过期时会返回 osErrorParameter
  * 且不延时，因此必须判返回值并在迟到后重新同步。
  ******************************************************************************
  */

#include "app/app_tasks.h"
#include "app/app_debug.h"
#include "app/mission.h"
#include "app/vision_link.h"
#include "arm/arm.h"
#include "chassis/chassis.h"
#include "chassis/chassis_config.h"
#include "common/keypad.h"

#include "cmsis_os.h"

void App_CarTask(void *argument)
{
    uint32_t next;

    (void)argument;

    next = osKernelGetTickCount();
    for (;;)
    {
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
        next += CHASSIS_CTRL_PERIOD_MS;

        Chassis_Control_Update();
    }
}

void App_EncTask(void *argument)
{
    uint32_t next;

    (void)argument;

    next = osKernelGetTickCount();
    for (;;)
    {
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
        next += CHASSIS_SENSE_PERIOD_MS;

        Chassis_Sense_Update();
    }
}

void App_MainTask(void *argument)
{
    uint32_t next;

    (void)argument;

    /* 上电静止 1s 后做陀螺零偏标定；结果显示在 OLED 页头 GOK/GERR。
       这是初始化阶段唯一的阻塞调用，之后主循环严格无阻塞。 */
    osDelay(1000);
    {
        int cal = Chassis_Calibrate(3000U);

        App_Debug_SetGyroCalib((cal == 0) ? 1U : 2U);

        /* 校准成功后把 Z 轴航向清零，消除上电后 0.3~0.7° 的初始漂移。
           失败时不清零，避免把未校准的漂移当作基准。 */
        if (cal == 0)
        {
            Chassis_ResetHeading();
        }
    }

#if (APP_STAGE != APP_STAGE_MISSION)
#if CHASSIS_AUTO_RUN_PATH
    (void)Chassis_Path_Start();
#endif
#endif
    /* 正式模式：校准完成后由按键 9 启动 Misson，不自动发车 */

    next = osKernelGetTickCount();
    for (;;)
    {
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
        next += KEYPAD_SCAN_PERIOD_MS;

        /* 单拍只推进一个扫描相位，立即返回 */
        Keypad_Scan();
        App_Debug_HandleKey(Keypad_GetEvent());
        App_Debug_Update();

        /* 视觉链路：解析已收帧、回 ACK、超时重发与失联判定 */
        Vision_Link_Update();

        /* 路径非阻塞推进 + 事件处理：跑图期间键盘与调试保持可用 */
        Chassis_Path_Update();

#if (APP_STAGE == APP_STAGE_MISSION)
        /* 正式模式：mission 独占路径/视觉事件；演示接管时交给调试层 */
        Mission_Update();
        if (Mission_OwnsPath() == 0U)
        {
            App_Debug_HandlePathEvents();
        }
#else
        App_Debug_HandlePathEvents();
#endif

        /* 机械臂由独立 ArmTask 推进，不占用本任务的节拍与栈 */
    }
}

void App_OledTask(void *argument)
{
    uint32_t next;

    (void)argument;

    next = osKernelGetTickCount();
    for (;;)
    {
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
        next += CHASSIS_DBG_PERIOD_MS;

        App_Debug_Refresh();
    }
}

void App_ArmTask(void *argument)
{
    uint32_t next;

    (void)argument;

    next = osKernelGetTickCount();
    for (;;)
    {
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
        next += CHASSIS_CTRL_PERIOD_MS;

        /* 机械臂状态机与阻塞串口发送都在本任务，不影响路径与键盘 */
        Arm_Update();
    }
}

void App_DefaultTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(1000);
    }
}
