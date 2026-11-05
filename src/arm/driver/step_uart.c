/**
  ******************************************************************************
  * @file    arm/driver/step_uart.c
  * @brief   ZDT X42S（Emm 固件）串口驱动。
  *
  * 帧格式：地址 + 功能码 + 数据 + 校验（默认固定 0x6B）。
  * 中断内只做字节解析与整数状态更新，浮点换算放在 Get 接口。
  ******************************************************************************
  */

#include "arm/driver/step_uart.h"
#include "arm/arm_config.h"
#include "common/uart_rx.h"
#include "main.h"

#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;

#define STEP_PENDING_NONE   (0U)
#define STEP_PENDING_ACK    (1U)
#define STEP_PENDING_POS    (2U)
#define STEP_PENDING_VER    (3U)

#define STEP_ERR_NONE       (0U)
#define STEP_ERR_TX         (1U)
#define STEP_ERR_TIMEOUT    (2U)
#define STEP_ERR_CHECK      (3U)
#define STEP_ERR_FUNC       (4U)
#define STEP_ERR_ACK        (5U)

#define STEP_FRAME_MAX      (16U)

/* 审计 H10：USART1 同样由 ArmTask 与 MainTask02 共用，发送段加锁。
   内核起来前为 NULL（初始化阶段单任务），不加锁也安全。 */
static osMutexId_t s_tx_mutex;

static uint8_t  s_rx_buf[STEP_FRAME_MAX];
static uint8_t  s_rx_len;
static uint8_t  s_rx_expect;

static volatile uint8_t  s_pending;
static volatile uint32_t s_tx_ms;
static volatile uint32_t s_pending_timeout_ms;
static volatile uint32_t s_last_rx_ms;
static volatile uint8_t  s_online;

static volatile uint8_t  s_pos_valid;
static volatile uint8_t  s_pos_sign;
static volatile uint32_t s_pos_raw;

static volatile uint8_t  s_ver_valid;
static volatile uint8_t  s_fw;
static volatile uint8_t  s_hw;

static volatile uint32_t s_err;
static volatile uint8_t  s_last_err_kind;
static volatile uint8_t  s_last_ack;

static uint8_t step_frame_len(uint8_t func)
{
    switch (func)
    {
    case 0x36U: return 8U;   /* 读实时位置 */
    case 0x1FU: return 5U;   /* 读固件/硬件版本 */
    case 0xF3U:              /* 使能 */
    case 0xFDU:              /* 位置模式 */
    case 0xFEU:              /* 立即停止 */
    case 0xFFU:              /* 同步触发 */
        return 4U;
    default:
        return 0U;
    }
}

static void step_rx_byte(uint8_t byte)
{
    uint8_t func;

    if (s_rx_len == 0U)
    {
        if (byte == ARM_STEP_ADDR)
        {
            s_rx_buf[0] = byte;
            s_rx_len = 1U;
        }
        return;
    }

    if (s_rx_len == 1U)
    {
        func = byte;
        s_rx_expect = step_frame_len(func);
        if (s_rx_expect == 0U)
        {
            s_rx_len = 0U;
            s_err++;
            s_last_err_kind = STEP_ERR_FUNC;
            return;
        }
        s_rx_buf[1] = byte;
        s_rx_len = 2U;
        return;
    }

    s_rx_buf[s_rx_len++] = byte;

    if (s_rx_len < s_rx_expect)
    {
        return;
    }

    if (s_rx_buf[s_rx_expect - 1U] != ARM_STEP_CHECK)
    {
        s_err++;
        s_last_err_kind = STEP_ERR_CHECK;
        s_rx_len = 0U;
        return;
    }

    s_online = 1U;
    s_last_rx_ms = HAL_GetTick();

    switch (s_rx_buf[1])
    {
    case 0x36U:
        s_pos_sign = s_rx_buf[2];
        s_pos_raw = ((uint32_t)s_rx_buf[3] << 24)
                  | ((uint32_t)s_rx_buf[4] << 16)
                  | ((uint32_t)s_rx_buf[5] << 8)
                  | (uint32_t)s_rx_buf[6];
        s_pos_valid = 1U;
        if (s_pending == STEP_PENDING_POS)
        {
            s_pending = STEP_PENDING_NONE;
        }
        break;

    case 0x1FU:
        s_fw = s_rx_buf[2];
        s_hw = s_rx_buf[3];
        s_ver_valid = 1U;
        if (s_pending == STEP_PENDING_VER)
        {
            s_pending = STEP_PENDING_NONE;
        }
        break;

    default:
        s_last_ack = s_rx_buf[2];

        /* 0x02 = 命令接收成功；0x9F = 动作完成通知，二者都不是错误 */
        if (s_rx_buf[2] != 0x02U && s_rx_buf[2] != 0x9FU)
        {
            s_err++;
            s_last_err_kind = STEP_ERR_ACK;
        }

        if (s_pending == STEP_PENDING_ACK)
        {
            s_pending = STEP_PENDING_NONE;
        }
        break;
    }

    s_rx_len = 0U;
}

static int step_begin(uint8_t pending)
{
    if (s_pending != STEP_PENDING_NONE)
    {
        return -1;
    }

    s_pending = pending;
    s_tx_ms = HAL_GetTick();
    s_pending_timeout_ms = ARM_STEP_TIMEOUT_MS;
    return 0;
}

static int step_send(const uint8_t *buf, uint8_t len)
{
    int rc = 0;

    if (s_tx_mutex != NULL)
    {
        (void)osMutexAcquire(s_tx_mutex, osWaitForever);
    }

    if (HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20U) != HAL_OK)
    {
        s_err++;
        s_last_err_kind = STEP_ERR_TX;
        rc = -1;
    }
    else
    {
        s_tx_ms = HAL_GetTick();
    }

    if (s_tx_mutex != NULL)
    {
        (void)osMutexRelease(s_tx_mutex);
    }

    return rc;
}

void Arm_Step_InitRtos(void)
{
    /* 必须在 osKernelInitialize() 之后调用（见 app_init.c） */
    if (s_tx_mutex == NULL)
    {
        s_tx_mutex = osMutexNew(NULL);
    }
}

void Arm_Step_Init(void)
{
    s_rx_len = 0U;
    s_rx_expect = 0U;
    s_pending = STEP_PENDING_NONE;
    s_tx_ms = 0U;
    s_pending_timeout_ms = ARM_STEP_TIMEOUT_MS;
    s_last_rx_ms = 0U;
    s_online = 0U;
    s_pos_valid = 0U;
    s_pos_sign = 0U;
    s_pos_raw = 0U;
    s_ver_valid = 0U;
    s_fw = 0U;
    s_hw = 0U;
    s_err = 0U;
    s_last_err_kind = STEP_ERR_NONE;
    s_last_ack = 0U;

    Uart_Rx_Register(&huart1, step_rx_byte);
    Uart_Rx_Start(&huart1);
}

void Arm_Step_Update(void)
{
    if (s_pending != STEP_PENDING_NONE
        && (HAL_GetTick() - s_tx_ms) > s_pending_timeout_ms)
    {
        s_pending = STEP_PENDING_NONE;
        s_err++;
        s_last_err_kind = STEP_ERR_TIMEOUT;
    }
}

int Arm_Step_Enable(uint8_t on)
{
    uint8_t buf[6];

    /* 使能/失能是安全相关命令，忽略未完成事务，直接下发 */
    s_pending = STEP_PENDING_ACK;
    s_tx_ms = HAL_GetTick();
    s_pending_timeout_ms = ARM_STEP_TIMEOUT_MS;

    buf[0] = ARM_STEP_ADDR;
    buf[1] = 0xF3U;
    buf[2] = 0xABU;
    buf[3] = (on != 0U) ? 1U : 0U;
    buf[4] = 0x00U;
    buf[5] = ARM_STEP_CHECK;

    return step_send(buf, sizeof(buf));
}

int Arm_Step_StopNow(void)
{
    uint8_t buf[5];

    /* 立即停止是安全命令，忽略未完成事务，直接下发 */
    s_pending = STEP_PENDING_ACK;
    s_tx_ms = HAL_GetTick();
    s_pending_timeout_ms = ARM_STEP_TIMEOUT_MS;

    buf[0] = ARM_STEP_ADDR;
    buf[1] = 0xFEU;
    buf[2] = 0x98U;
    buf[3] = 0x00U;
    buf[4] = ARM_STEP_CHECK;

    return step_send(buf, sizeof(buf));
}

int Arm_Step_MoveRelativePulses(int32_t pulses, uint16_t speed_rpm)
{
    uint8_t  buf[13];
    uint32_t mag;
    uint16_t speed;
    uint8_t  dir;

    if (pulses == 0)
    {
        return -1;
    }

    if (pulses > 0)
    {
        mag = (uint32_t)pulses;
        dir = 0U;
    }
    else
    {
        mag = (uint32_t)(-(pulses + 1)) + 1U;
        dir = 1U;
    }

#if ARM_STEP_DIR_INVERT
    dir = (dir != 0U) ? 0U : 1U;
#endif

    speed = (speed_rpm == 0U) ? (uint16_t)ARM_STEP_SPEED_RPM : speed_rpm;

    if (step_begin(STEP_PENDING_ACK) != 0)
    {
        return -1;
    }

    /* 位置模式的完成通知在运动结束后才到，按运动时间动态等待 */
    {
        float revs = (float)mag / ARM_STEP_PULSES_PER_REV;
        float sec = revs * 60.0f / (float)speed;
        uint32_t t_ms = (uint32_t)(sec * 1000.0f) + 500U;

        if (t_ms < ARM_STEP_TIMEOUT_MS)
        {
            t_ms = ARM_STEP_TIMEOUT_MS;
        }
        s_pending_timeout_ms = t_ms;
    }

    buf[0]  = ARM_STEP_ADDR;
    buf[1]  = 0xFDU;
    buf[2]  = dir;
    buf[3]  = (uint8_t)(speed >> 8);
    buf[4]  = (uint8_t)(speed & 0xFFU);
    buf[5]  = (uint8_t)ARM_STEP_ACCEL;
    buf[6]  = (uint8_t)(mag >> 24);
    buf[7]  = (uint8_t)(mag >> 16);
    buf[8]  = (uint8_t)(mag >> 8);
    buf[9]  = (uint8_t)(mag & 0xFFU);
    buf[10] = 0x00U;                 /* 相对位置 */
    buf[11] = 0x00U;                 /* 不使用多机同步 */
    buf[12] = ARM_STEP_CHECK;

    return step_send(buf, sizeof(buf));
}

int Arm_Step_SyncTrigger(void)
{
    uint8_t buf[4];

    buf[0] = 0x00U;                  /* 广播地址 */
    buf[1] = 0xFFU;
    buf[2] = 0x66U;
    buf[3] = ARM_STEP_CHECK;

    return step_send(buf, sizeof(buf));
}

void Arm_Step_RequestPosition(void)
{
    uint8_t buf[3];

    if (step_begin(STEP_PENDING_POS) != 0)
    {
        return;
    }

    buf[0] = ARM_STEP_ADDR;
    buf[1] = 0x36U;
    buf[2] = ARM_STEP_CHECK;

    (void)step_send(buf, sizeof(buf));
}

int Arm_Step_GetPositionDeg(float *deg)
{
    float value;

    if (deg == NULL || s_pos_valid == 0U)
    {
        return -1;
    }

    value = (float)s_pos_raw * (360.0f / 65536.0f);
    if (s_pos_sign != 0U)
    {
        value = -value;
    }

    *deg = value;
    return 0;
}

void Arm_Step_RequestVersion(void)
{
    uint8_t buf[3];

    if (step_begin(STEP_PENDING_VER) != 0)
    {
        return;
    }

    buf[0] = ARM_STEP_ADDR;
    buf[1] = 0x1FU;
    buf[2] = ARM_STEP_CHECK;

    (void)step_send(buf, sizeof(buf));
}

int Arm_Step_GetVersion(uint8_t *fw, uint8_t *hw)
{
    if (fw == NULL || hw == NULL || s_ver_valid == 0U)
    {
        return -1;
    }

    *fw = s_fw;
    *hw = s_hw;
    return 0;
}

uint8_t Arm_Step_IsOnline(void)
{
    return (s_online != 0U
            && (HAL_GetTick() - s_last_rx_ms) < ARM_STEP_ONLINE_MS) ? 1U : 0U;
}

uint32_t Arm_Step_GetErrorCount(void)
{
    return s_err;
}

uint8_t Arm_Step_GetLastErrorKind(void)
{
    return s_last_err_kind;
}

uint8_t Arm_Step_GetLastAck(void)
{
    return s_last_ack;
}
