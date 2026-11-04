/**
  ******************************************************************************
  * @file    chassis/driver/gyro.c
  * @brief   JY61P（WIT 协议）解析与状态维护。
  *
  * 中断内只做整数解析与状态更新，所有浮点换算放到任务上下文的 Get 接口。
  ******************************************************************************
  */

#include "chassis/driver/gyro.h"
#include "chassis/chassis_config.h"
#include "common/uart_rx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart2;

#define GYRO_FRAME_MAX   (16U)
#define GYRO_ST_HEAD     (0U)
#define GYRO_ST_FLAG     (1U)
#define GYRO_ST_DATA     (2U)
#define GYRO_ST_SUM      (3U)

#define GYRO_WZ_LSB_DPS  (2000.0f / 32768.0f)
#define GYRO_YAW_LSB_DEG (180.0f / 32768.0f)

static uint8_t s_buf[GYRO_FRAME_MAX];
static uint8_t s_idx;
static uint8_t s_len;
static uint8_t s_sum;
static uint8_t s_state;

static volatile int16_t  s_raw_yaw;
static volatile int16_t  s_raw_wz;
static volatile uint32_t s_tick;
static volatile uint32_t s_seq;
static volatile uint32_t s_err;
static volatile uint32_t s_err_head;
static volatile uint32_t s_err_flag;
static volatile uint32_t s_err_sum;
static volatile uint32_t s_yaw_frames;
static volatile uint32_t s_wz_frames;
static volatile uint32_t s_rx_bytes;
static volatile uint8_t  s_has_yaw;
static volatile uint8_t  s_has_wz;
static volatile uint8_t  s_new_frame;
static volatile uint8_t  s_synced;        /* 当前是否与帧边界同步 */
static volatile uint8_t  s_last_bad_flag; /* 最近一次未定义的标志位 */

static volatile int32_t  s_calib_sum;
static volatile uint32_t s_calib_cnt;
static uint32_t          s_calib_target;
static volatile uint8_t  s_calib_active;
static volatile uint8_t  s_calib_done;
static float             s_bias_wz;
static float             s_yaw_scale = 1.0f;

static uint8_t wit_frame_len(uint8_t flag)
{
    switch (flag)
    {
    case 0x50U:                 /* 时间 */
        return 15U;
    case 0x51U:                 /* 加速度 */
    case 0x52U:                 /* 角速度 */
    case 0x53U:                 /* 角度 */
    case 0x54U:                 /* 磁场（部分固件仍输出） */
        return 11U;
    case 0x55U:                 /* 端口状态 */
    case 0x56U:                 /* 气压高度 */
    case 0x57U:                 /* 经纬度 */
    case 0x58U:                 /* GPS */
    case 0x59U:                 /* 四元数 */
        return 15U;
    default:
        return 0U;
    }
}

static void gyro_parse_frame(const uint8_t *b)
{
    int16_t raw;

    if (b[1] == 0x53U)
    {
        raw = (int16_t)(((uint16_t)b[7] << 8) | (uint16_t)b[6]);
        s_raw_yaw = raw;
        s_has_yaw = 1U;
        s_tick = HAL_GetTick();
        s_seq++;
        s_yaw_frames++;
        s_new_frame = 1U;
    }
    else if (b[1] == 0x52U)
    {
        raw = (int16_t)(((uint16_t)b[7] << 8) | (uint16_t)b[6]);
        s_raw_wz = raw;
        s_has_wz = 1U;
        s_tick = HAL_GetTick();
        s_seq++;
        s_wz_frames++;
        s_new_frame = 1U;

        if (s_calib_active != 0U && s_calib_done == 0U)
        {
            s_calib_sum += (int32_t)raw;
            s_calib_cnt++;
        }
    }
    else
    {
        /* 0x51 加速度帧本轮不使用 */
    }
}

void Chassis_Gyro_RxByteISR(uint8_t byte)
{
    s_rx_bytes++;

    switch (s_state)
    {
    case GYRO_ST_HEAD:
        if (byte == 0x55U)
        {
            s_buf[0] = byte;
            s_idx = 1U;
            s_synced = 1U;
            s_state = GYRO_ST_FLAG;
        }
        else
        {
            /* 一次连续失步只计一次，避免一帧数据被重复统计 */
            if (s_synced != 0U)
            {
                s_err_head++;
                s_err++;
                s_synced = 0U;
            }
        }
        break;

    case GYRO_ST_FLAG:
        s_len = wit_frame_len(byte);
        if (s_len == 0U)
        {
            s_last_bad_flag = byte;
            s_err_flag++;
            s_err++;
            s_synced = 0U;
            s_state = GYRO_ST_HEAD;
        }
        else
        {
            s_buf[1] = byte;
            s_sum = (uint8_t)(0x55U + byte);
            s_idx = 2U;
            s_state = GYRO_ST_DATA;
        }
        break;

    case GYRO_ST_DATA:
        s_buf[s_idx++] = byte;
        s_sum = (uint8_t)(s_sum + byte);
        if (s_idx >= (uint8_t)(s_len - 1U))
        {
            s_state = GYRO_ST_SUM;
        }
        break;

    case GYRO_ST_SUM:
        if ((uint8_t)(s_sum & 0xFFU) == byte)
        {
            gyro_parse_frame(s_buf);
            s_synced = 1U;
        }
        else
        {
            s_err_sum++;
            s_err++;
            s_synced = 0U;
        }
        s_state = GYRO_ST_HEAD;
        break;

    default:
        s_state = GYRO_ST_HEAD;
        break;
    }
}

void Chassis_Gyro_Init(void)
{
    s_state = GYRO_ST_HEAD;
    s_idx = 0U;
    s_len = 0U;
    s_sum = 0U;

    s_raw_yaw = 0;
    s_raw_wz = 0;
    s_tick = 0U;
    s_seq = 0U;
    s_err = 0U;
    s_err_head = 0U;
    s_err_flag = 0U;
    s_err_sum = 0U;
    s_yaw_frames = 0U;
    s_wz_frames = 0U;
    s_rx_bytes = 0U;
    s_last_bad_flag = 0U;
    s_synced = 1U;
    s_has_yaw = 0U;
    s_has_wz = 0U;
    s_new_frame = 0U;

    s_calib_sum = 0;
    s_calib_cnt = 0U;
    s_calib_target = 0U;
    s_calib_active = 0U;
    s_calib_done = 0U;
    s_bias_wz = 0.0f;
    s_yaw_scale = 1.0f;

#if CHASSIS_GYRO_AUTO_CONFIG
    /* 提高输出速率。只写寄存器、不发保存命令，避免每次上电擦写模块 Flash。
       解锁值 0x88B5 对应命令 FF AA 69 88 B5。 */
    Chassis_Gyro_SendCmd(CHASSIS_GYRO_REG_UNLOCK, 0xB588U);
    HAL_Delay(200U);
    Chassis_Gyro_SendCmd(CHASSIS_GYRO_REG_RATE, CHASSIS_GYRO_RATE_VALUE);
#endif

    Uart_Rx_Register(&huart2, Chassis_Gyro_RxByteISR);
    Uart_Rx_Start(&huart2);
}

static void gyro_update_bias(void)
{
    if (s_calib_active != 0U && s_calib_done == 0U
        && s_calib_cnt >= s_calib_target && s_calib_cnt > 0U)
    {
        float mean_raw = (float)s_calib_sum / (float)s_calib_cnt;
        s_bias_wz = mean_raw * GYRO_WZ_LSB_DPS * CHASSIS_GYRO_WZ_SIGN;
        s_calib_active = 0U;
        s_calib_done = 1U;
    }
}

uint8_t Chassis_Gyro_GetFrame(Chassis_Gyro_Frame_t *out)
{
    uint8_t is_new;

    if (out == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    is_new = s_new_frame;
    s_new_frame = 0U;

    out->yaw_raw = s_raw_yaw;
    out->wz_raw = s_raw_wz;
    out->tick_ms = s_tick;
    out->seq = s_seq;
    out->rx_bytes = s_rx_bytes;
    out->yaw_frames = s_yaw_frames;
    out->wz_frames = s_wz_frames;
    out->calib_cnt = s_calib_cnt;
    out->calib_target = s_calib_target;
    out->err_head = s_err_head;
    out->err_flag = s_err_flag;
    out->err_sum = s_err_sum;
    out->last_bad_flag = s_last_bad_flag;
    out->has_yaw = s_has_yaw;
    out->has_wz = s_has_wz;
    taskEXIT_CRITICAL();

    out->yaw_deg = (float)out->yaw_raw * GYRO_YAW_LSB_DEG
                 * CHASSIS_GYRO_YAW_SIGN * s_yaw_scale;
    out->wz_dps = (float)out->wz_raw * GYRO_WZ_LSB_DPS
                * CHASSIS_GYRO_WZ_SIGN - s_bias_wz;

    gyro_update_bias();

    return is_new;
}

void Chassis_Gyro_StartBiasCalib(uint16_t n_samples)
{
    taskENTER_CRITICAL();
    s_calib_sum = 0;
    s_calib_cnt = 0U;
    s_calib_target = n_samples;
    s_calib_done = 0U;
    s_calib_active = 1U;
    taskEXIT_CRITICAL();
}

uint8_t Chassis_Gyro_IsCalibDone(void)
{
    /* 标定完成判定在此触发，避免依赖 GetFrame 的调用频率 */
    gyro_update_bias();
    return s_calib_done;
}

void Chassis_Gyro_SetYawScale(float k)
{
    if (k > 0.1f && k < 10.0f)
    {
        s_yaw_scale = k;
    }
}

float Chassis_Gyro_GetYawDeg(void)
{
    int16_t raw;

    taskENTER_CRITICAL();
    raw = s_raw_yaw;
    taskEXIT_CRITICAL();

    return (float)raw * GYRO_YAW_LSB_DEG
         * CHASSIS_GYRO_YAW_SIGN * s_yaw_scale;
}

float Chassis_Gyro_GetWzDps(void)
{
    int16_t raw;

    taskENTER_CRITICAL();
    raw = s_raw_wz;
    taskEXIT_CRITICAL();

    return (float)raw * GYRO_WZ_LSB_DPS * CHASSIS_GYRO_WZ_SIGN - s_bias_wz;
}

uint8_t Chassis_Gyro_IsOnline(void)
{
    uint32_t tick;
    uint32_t seq;

    taskENTER_CRITICAL();
    tick = s_tick;
    seq = s_seq;
    taskEXIT_CRITICAL();

    if (seq == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - tick) < CHASSIS_GYRO_TIMEOUT_MS) ? 1U : 0U;
}

uint32_t Chassis_Gyro_GetErrorCount(void)
{
    return s_err;
}

void Chassis_Gyro_SendCmd(uint8_t reg, uint16_t val)
{
    uint8_t cmd[5];

    cmd[0] = 0xFFU;
    cmd[1] = 0xAAU;
    cmd[2] = reg;
    cmd[3] = (uint8_t)(val & 0xFFU);
    cmd[4] = (uint8_t)(val >> 8);

    HAL_UART_Transmit(&huart2, cmd, 5U, 100U);
}
