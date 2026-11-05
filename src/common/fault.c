/**
  ******************************************************************************
  * @file    common/fault.c
  * @brief   故障记录：把故障码/出错任务名写进 BKP 备份寄存器，供复位后查看。
  *
  * 审计 H8：Core/Src/freertos.c 里的 vApplicationStackOverflowHook /
  * vApplicationMallocFailedHook 原来只 bkpt + 关中断死等——没有调试器时现场
  * 什么也留不下。这两条钩子现在实现在 freertos.c 的 USER CODE 区并调用本文件
  * 的 Fault_Record/Fault_Halt（记录落进 BKP，复位后开机可读）。
  * 仓库里没有 printf（栈小、无 _write），所以不打印，只落盘 + OLED 显示。
  ******************************************************************************
  */

#include "common/fault.h"
#include "main.h"

/* 开机时读出的上一条记录 */
static uint16_t s_last_code;
static char     s_last_name[5];

static void fault_bkp_unlock(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

void Fault_Init(void)
{
    uint16_t lo;
    uint16_t hi;

    s_last_code   = FAULT_CODE_NONE;
    s_last_name[0] = '\0';

    fault_bkp_unlock();

    lo = BKP->DR1;
    if (lo != FAULT_CODE_NONE)
    {
        hi = BKP->DR2;
        s_last_name[0] = (char)(hi & 0xFFU);
        s_last_name[1] = (char)((hi >> 8) & 0xFFU);
        hi = BKP->DR3;
        s_last_name[2] = (char)(hi & 0xFFU);
        s_last_name[3] = (char)((hi >> 8) & 0xFFU);
        s_last_name[4] = '\0';

        s_last_code = lo;

        /* 读过即清零，下一条记录才是新的；DR5 的连续性计数不动 */
        BKP->DR1 = FAULT_CODE_NONE;
        BKP->DR2 = 0U;
        BKP->DR3 = 0U;
        BKP->DR4 = 0U;
    }
}

void Fault_Record(uint16_t code, const char *task_name)
{
    uint16_t lo = 0U;
    uint16_t hi = 0U;

    fault_bkp_unlock();

    if (task_name != NULL)
    {
        /* 只存前 4 个字符：MainTask02/CarTask/EncTask/ArmTask/OLEDTask/
           defaultTask 的前 4 字符互不重复 */
        if (task_name[0] != '\0')
        {
            lo |= (uint16_t)(uint8_t)task_name[0];
        }
        if (task_name[1] != '\0')
        {
            lo |= (uint16_t)((uint16_t)(uint8_t)task_name[1] << 8);
        }
        if (task_name[2] != '\0')
        {
            hi |= (uint16_t)(uint8_t)task_name[2];
        }
        if (task_name[3] != '\0')
        {
            hi |= (uint16_t)((uint16_t)(uint8_t)task_name[3] << 8);
        }
    }

    BKP->DR2 = lo;
    BKP->DR3 = hi;
    BKP->DR4 = (uint16_t)(HAL_GetTick() & 0xFFFFU);
    BKP->DR1 = code;      /* 最后写：DR1 非 0 才表示记录完整 */
}

uint16_t Fault_GetLastCode(void)
{
    return s_last_code;
}

const char *Fault_GetLastTaskName(void)
{
    return s_last_name;
}

void Fault_Halt(void)
{
    __disable_irq();

    /* 不喂狗：IWDG 启动过就在 ≤2s 后复位（记录已在 BKP 里）；
       调试模式（DEBUG_WITHOUT_IWDG）下则停在这里，断点/看变量即可 */
    for (;;)
    {
    }
}

void Fault_Fatal(uint16_t code)
{
    Fault_Record(code, NULL);

    BKP->DR5 = (uint16_t)(BKP->DR5 + 1U);

    /* 连续 3 次都是「还没进入正常调度就死」，就别再来一次复位风暴；
       停在原地等调试器/人工断电。计数在 Watchdog_Start() 里清零。 */
    if (BKP->DR5 < 3U)
    {
        NVIC_SystemReset();
    }

    Fault_Halt();
}
