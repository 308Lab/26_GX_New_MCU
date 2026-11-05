/**
  ******************************************************************************
  * @file    common/watchdog.c
  * @brief   任务级 IWDG 喂狗（审计 H9）。
  *
  * 各任务的喂狗窗口按「最大允许停摆时间」取，而不是任务周期——窗口太紧会把
  * 偶发的一次长拍（浮点运算抢 CPU、串口阻塞发送、mutex 等待）误判成任务卡死，
  * 那就是跑图途中莫名复位：
  *   Main/Car/Enc 都是 5 ms 拍，窗口 100 ms（20 个拍）；
  *   Arm 一拍里可能发一帧阻塞串口（约 7 ms）+ 一次回读，窗口 200 ms；
  *   OLED 200 ms 一拍、软 I2C 刷 128x64 要几十 ms，窗口 800 ms。
  * IWDG 超时取 2 s：比最长窗口（800 ms）大得多，不会误复位，又能在 2 s 内
  * 把卡死的系统拉回来。
  ******************************************************************************
  */

#include "common/watchdog.h"
#include "common/fault.h"
#include "main.h"

#if !defined(DEBUG_WITHOUT_IWDG)
static IWDG_HandleTypeDef s_hiwdg;
#endif

static uint32_t s_kick_ms[WD_NUM];
static uint8_t  s_started;
static uint8_t  s_stale_logged;

static const uint32_t s_window_ms[WD_NUM] =
{
    100U,   /* WD_MAIN */
    100U,   /* WD_CAR  */
    100U,   /* WD_ENC  */
    200U,   /* WD_ARM  */
    800U    /* WD_OLED */
};

static const char *const s_name[WD_NUM] =
{
    "Main", "Car", "Enc", "Arm", "OL"
};

void Watchdog_Start(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    for (i = 0U; i < (uint8_t)WD_NUM; i++)
    {
        s_kick_ms[i] = now;
    }
    s_stale_logged = 0U;
    s_started = 0U;

#if defined(DEBUG_WITHOUT_IWDG)
    /* 调试模式：不启动 IWDG，断点/单步不会被复位 */
#else
    s_hiwdg.Instance          = IWDG;
    s_hiwdg.Init.Prescaler    = IWDG_PRESCALER_256;   /* LSI 40kHz/256 = 6.4 ms 一拍 */
    s_hiwdg.Init.Reload       = 312U;                 /* 312 * 6.4 ms ≈ 2.0 s */

    if (HAL_IWDG_Init(&s_hiwdg) != HAL_OK)
    {
        Fault_Fatal(FAULT_CODE_ERROR_HANDLER);
    }
    HAL_IWDG_Refresh(&s_hiwdg);

    s_started = 1U;
#endif

    /* 系统真的跑起来了：清掉「连续致命复位」计数 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    BKP->DR5 = 0U;
}

void Watchdog_Kick(Wd_Owner_e owner)
{
    if ((uint8_t)owner < (uint8_t)WD_NUM)
    {
        s_kick_ms[owner] = HAL_GetTick();
    }
}

uint8_t Watchdog_Service(void)
{
    uint32_t now;
    uint8_t  i;

    if (s_started == 0U)
    {
        return 0U;
    }

    now = HAL_GetTick();

    for (i = 0U; i < (uint8_t)WD_NUM; i++)
    {
        if ((uint32_t)(now - s_kick_ms[i]) > s_window_ms[i])
        {
            if (s_stale_logged == 0U)
            {
                s_stale_logged = 1U;
                /* 先落盘是谁卡死了，再停止喂狗：IWDG 2s 后复位，
                   Fault_Init() 会把它读到 OLED 上 */
                Fault_Record(FAULT_CODE_WDG_STALE, s_name[i]);
            }

            return 0U;
        }
    }

#if defined(DEBUG_WITHOUT_IWDG)
    return 1U;
#else
    HAL_IWDG_Refresh(&s_hiwdg);
    return 1U;
#endif
}

uint8_t Watchdog_IsRunning(void)
{
    return s_started;
}
