/**
  ******************************************************************************
  * @file    arm/driver/servo_bus.c
  * @brief   众灵 ZX361S 总线舵机 ASCII 协议驱动（UART4 半双工）。
  *
  * 写指令无回复；只有 PRAD 回读产生应答。收发方向由 PC9/PA11 切换。
  ******************************************************************************
  */

#include "arm/driver/servo_bus.h"
#include "arm/arm_config.h"
#include "common/uart_rx.h"
#include "main.h"

extern UART_HandleTypeDef huart4;

#define SERVO_SLOT_NUM      (3U)
#define SERVO_PENDING_NONE  (0xFFU)
#define SERVO_LINE_MAX      (24U)
#define SERVO_TX_MAX        (80U)

static const uint8_t s_ids[SERVO_SLOT_NUM] =
{
    (uint8_t)ARM_SERVO_ID_J2,
    (uint8_t)ARM_SERVO_ID_J3,
    (uint8_t)ARM_SERVO_ID_J4
};

static volatile uint16_t s_pwm[SERVO_SLOT_NUM];
static volatile uint8_t  s_pwm_valid[SERVO_SLOT_NUM];
static volatile uint8_t  s_online[SERVO_SLOT_NUM];
static volatile uint8_t  s_torque_on[SERVO_SLOT_NUM];
static volatile uint32_t s_last_rx_ms[SERVO_SLOT_NUM];
static volatile uint32_t s_err[SERVO_SLOT_NUM];

static volatile uint8_t s_pending_slot;
static volatile uint32_t s_tx_ms;

/* 任意 ID 的位置回读（用于 ID 0 或未映射的地址） */
static volatile uint8_t  s_any_pending;
static volatile uint8_t  s_any_id;
static volatile uint8_t  s_any_valid;
static volatile uint16_t s_any_pwm;
static volatile uint32_t s_any_tx_ms;

static volatile uint8_t  s_last_id;
static volatile uint8_t  s_id_valid;

/* 0 = PC9 作 TX、PA11 作 RX；1 = 交换（扫描失败时自动重试） */
static uint8_t s_dir_swap;
static volatile uint8_t  s_id_pending;   /* 0xFF = 无 */
static volatile uint32_t s_id_tx_ms;
static volatile uint8_t  s_id_result;    /* 0=等待/无，1=成功，2=失败 */
static volatile uint8_t  s_bcast_pending;

#define SERVO_SCAN_MAX  (16U)
static uint8_t s_scan_active;
static uint8_t s_scan_done;
static uint8_t s_scan_id;
static uint8_t s_scan_first;
static uint8_t s_scan_last;
static uint8_t s_scan_retried;
static uint8_t s_scan_count;
static uint8_t s_scan_ids[SERVO_SCAN_MAX];

static char    s_line[SERVO_LINE_MAX];
static uint8_t s_line_len;
static uint8_t s_line_active;

static int8_t servo_slot_of_id(uint8_t id)
{
    uint8_t i;

    for (i = 0U; i < SERVO_SLOT_NUM; i++)
    {
        if (s_ids[i] == id)
        {
            return (int8_t)i;
        }
    }

    return -1;
}

static void servo_set_dir(uint8_t tx)
{
    uint8_t tx_level = (ARM_SERVO_TX_EN_LEVEL != 0U) ? tx : (uint8_t)(tx ^ 1U);
    uint8_t rx_level = (ARM_SERVO_RX_EN_LEVEL != 0U) ? (uint8_t)(tx ^ 1U) : tx;

    GPIO_TypeDef *tx_port = (s_dir_swap != 0U)
                          ? Bsvo_RxSwitch_GPIO_Port : Bsvo_TxSwitch_GPIO_Port;
    uint16_t tx_pin = (s_dir_swap != 0U)
                    ? Bsvo_RxSwitch_Pin : Bsvo_TxSwitch_Pin;
    GPIO_TypeDef *rx_port = (s_dir_swap != 0U)
                          ? Bsvo_TxSwitch_GPIO_Port : Bsvo_RxSwitch_GPIO_Port;
    uint16_t rx_pin = (s_dir_swap != 0U)
                    ? Bsvo_TxSwitch_Pin : Bsvo_RxSwitch_Pin;

    HAL_GPIO_WritePin(tx_port, tx_pin,
                      tx_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(rx_port, rx_pin,
                      rx_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void servo_parse_line(void)
{
    uint8_t  id = 0U;
    uint16_t pwm = 0U;
    int8_t   slot;
    uint8_t  i;

    if (s_line_len < 6U
        || s_line[0] != '#'
        || s_line[4] != 'P')
    {
        return;
    }

    id = (uint8_t)((s_line[1] - '0') * 100
                 + (s_line[2] - '0') * 10
                 + (s_line[3] - '0'));

    /* 读 ID 应答：#IDP! */
    if (s_line_len == 6U && s_line[5] == '!')
    {
        s_last_id = id;
        s_id_valid = 1U;

        if (s_id_pending == id || s_bcast_pending != 0U)
        {
            s_id_result = 1U;
            s_id_pending = 0xFFU;
            s_bcast_pending = 0U;
        }

        return;
    }

    /* 读位置应答：#IDPxxxx!；排除 #IDPVx.y! 等版本帧 */
    if (s_line_len < 10U || s_line[9] != '!')
    {
        return;
    }

    for (i = 5U; i < 9U; i++)
    {
        if (s_line[i] < '0' || s_line[i] > '9')
        {
            return;
        }
    }

    pwm = (uint16_t)((s_line[5] - '0') * 1000
                   + (s_line[6] - '0') * 100
                   + (s_line[7] - '0') * 10
                   + (s_line[8] - '0'));

    if (s_any_pending != 0U && s_any_id == id)
    {
        s_any_pwm = pwm;
        s_any_valid = 1U;
        s_any_pending = 0U;
    }

    slot = servo_slot_of_id(id);
    if (slot < 0)
    {
        return;
    }

    s_pwm[slot] = pwm;
    s_pwm_valid[slot] = 1U;
    s_online[slot] = 1U;
    s_last_rx_ms[slot] = HAL_GetTick();

    if (s_pending_slot == (uint8_t)slot)
    {
        s_pending_slot = SERVO_PENDING_NONE;
    }
}

static void servo_rx_byte(uint8_t byte)
{
    if (byte == (uint8_t)'#')
    {
        s_line_active = 1U;
        s_line_len = 0U;
    }

    if (s_line_active == 0U)
    {
        return;
    }

    if (s_line_len < (SERVO_LINE_MAX - 1U))
    {
        s_line[s_line_len++] = (char)byte;
    }
    else
    {
        s_line_active = 0U;
        return;
    }

    if (byte == (uint8_t)'!')
    {
        s_line[s_line_len] = '\0';
        s_line_active = 0U;
        servo_parse_line();
    }
}

static int servo_send(const char *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    servo_set_dir(1U);
    HAL_Delay(ARM_SERVO_SWITCH_DELAY_MS);

    st = HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 20U);

    servo_set_dir(0U);
    HAL_Delay(ARM_SERVO_SWITCH_DELAY_MS);

    return (st == HAL_OK) ? 0 : -1;
}

static char *put_u3(char *p, uint16_t v)
{
    *p++ = (char)('0' + (v / 100U) % 10U);
    *p++ = (char)('0' + (v / 10U) % 10U);
    *p++ = (char)('0' + v % 10U);
    return p;
}

static char *put_u4(char *p, uint16_t v)
{
    *p++ = (char)('0' + (v / 1000U) % 10U);
    *p++ = (char)('0' + (v / 100U) % 10U);
    *p++ = (char)('0' + (v / 10U) % 10U);
    *p++ = (char)('0' + v % 10U);
    return p;
}

static uint16_t servo_clamp_pwm(uint16_t pwm)
{
    if (pwm < ARM_SERVO_PWM_MIN)
    {
        return ARM_SERVO_PWM_MIN;
    }
    if (pwm > ARM_SERVO_PWM_MAX)
    {
        return ARM_SERVO_PWM_MAX;
    }
    return pwm;
}

static void servo_cancel_pending(void)
{
    s_pending_slot = SERVO_PENDING_NONE;
    s_any_pending = 0U;
    s_bcast_pending = 0U;
}

int Arm_Servo_SetPwm(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char    buf[16];
    char   *p = buf;

    pwm = servo_clamp_pwm(pwm);
    if (time_ms > 9999U)
    {
        time_ms = 9999U;
    }

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    p = put_u4(p, pwm);
    *p++ = 'T';
    p = put_u4(p, time_ms);
    *p++ = '!';

    return servo_send(buf, (uint16_t)(p - buf));
}

int Arm_Servo_SetPwmSync(const uint8_t *ids, const uint16_t *pwm,
                         uint8_t n, uint16_t time_ms)
{
    char    buf[SERVO_TX_MAX];
    char   *p = buf;
    uint8_t i;

    if (ids == NULL || pwm == NULL || n == 0U || n > SERVO_SLOT_NUM)
    {
        return -1;
    }

    if (time_ms > 9999U)
    {
        time_ms = 9999U;
    }

    servo_cancel_pending();

    p = buf;
    *p++ = '{';
    *p++ = 'G';
    p = put_u4(p, 0U);

    for (i = 0U; i < n; i++)
    {
        *p++ = '#';
        p = put_u3(p, ids[i]);
        *p++ = 'P';
        p = put_u4(p, servo_clamp_pwm(pwm[i]));
        *p++ = 'T';
        p = put_u4(p, time_ms);
        *p++ = '!';
    }

    *p++ = '}';

    return servo_send(buf, (uint16_t)(p - buf));
}

void Arm_Servo_RequestPosition(uint8_t id)
{
    char  buf[12];
    char *p = buf;
    int8_t slot = servo_slot_of_id(id);
    uint32_t now = HAL_GetTick();

    if (s_pending_slot != SERVO_PENDING_NONE
        || s_any_pending != 0U
        || s_bcast_pending != 0U)
    {
        return;
    }

    if (slot >= 0)
    {
        s_pending_slot = (uint8_t)slot;
        s_tx_ms = now;
    }
    else
    {
        s_any_id = id;
        s_any_valid = 0U;
        s_any_pending = 1U;
        s_any_tx_ms = now;
    }

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    *p++ = 'R';
    *p++ = 'A';
    *p++ = 'D';
    *p++ = '!';

    (void)servo_send(buf, (uint16_t)(p - buf));
}

void Arm_Servo_ClearPositionCache(void)
{
    uint8_t i;

    for (i = 0U; i < SERVO_SLOT_NUM; i++)
    {
        s_pwm_valid[i] = 0U;
    }

    s_any_valid = 0U;
}

int Arm_Servo_GetPositionPwm(uint8_t id, uint16_t *pwm)
{
    int8_t slot = servo_slot_of_id(id);

    if (pwm == NULL)
    {
        return -1;
    }

    if (slot >= 0)
    {
        if (s_pwm_valid[slot] == 0U)
        {
            return -1;
        }
        *pwm = s_pwm[slot];
        return 0;
    }

    if (s_any_id != id || s_any_valid == 0U)
    {
        return -1;
    }

    *pwm = s_any_pwm;
    return 0;
}

void Arm_Servo_Torque(uint8_t id, uint8_t on)
{
    char  buf[12];
    char *p = buf;
    int8_t slot = servo_slot_of_id(id);

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    *p++ = 'U';
    *p++ = 'L';
    *p++ = on ? 'R' : 'K';
    *p++ = '!';

    (void)servo_send(buf, (uint16_t)(p - buf));

    if (slot >= 0)
    {
        s_torque_on[slot] = (on != 0U) ? 1U : 0U;
    }
}

uint8_t Arm_Servo_GetTorque(uint8_t id)
{
    int8_t slot = servo_slot_of_id(id);

    return (slot < 0) ? 0U : s_torque_on[slot];
}

void Arm_Servo_TorqueBroadcast(uint8_t on)
{
    char  buf[12];
    char *p = buf;
    uint8_t i;

    servo_cancel_pending();
    s_id_pending = 0xFFU;

    *p++ = '#';
    *p++ = '2';
    *p++ = '5';
    *p++ = '5';
    *p++ = 'P';
    *p++ = 'U';
    *p++ = 'L';
    *p++ = on ? 'R' : 'K';
    *p++ = '!';

    (void)servo_send(buf, (uint16_t)(p - buf));

    for (i = 0U; i < SERVO_SLOT_NUM; i++)
    {
        s_torque_on[i] = (on != 0U) ? 1U : 0U;
    }
}

int Arm_Servo_SetStartPosition(uint8_t id)
{
    char  buf[12];
    char *p = buf;

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    *p++ = 'C';
    *p++ = 'S';
    *p++ = 'D';
    *p++ = '!';

    return servo_send(buf, (uint16_t)(p - buf));
}

void Arm_Servo_Stop(uint8_t id)
{
    char  buf[12];
    char *p = buf;

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    *p++ = 'D';
    *p++ = 'S';
    *p++ = 'T';
    *p++ = '!';

    (void)servo_send(buf, (uint16_t)(p - buf));
}

void Arm_Servo_StopAll(void)
{
    uint8_t i;

    for (i = 0U; i < SERVO_SLOT_NUM; i++)
    {
        Arm_Servo_Stop(s_ids[i]);
    }
}

void Arm_Servo_Init(void)
{
    uint8_t i;

    for (i = 0U; i < SERVO_SLOT_NUM; i++)
    {
        s_pwm[i] = 0U;
        s_pwm_valid[i] = 0U;
        s_online[i] = 0U;
        s_torque_on[i] = 1U;
        s_last_rx_ms[i] = 0U;
        s_err[i] = 0U;
    }

    s_pending_slot = SERVO_PENDING_NONE;
    s_tx_ms = 0U;
    s_any_pending = 0U;
    s_any_id = 0U;
    s_any_valid = 0U;
    s_any_pwm = 0U;
    s_any_tx_ms = 0U;
    s_last_id = 0U;
    s_id_valid = 0U;
    s_id_pending = 0xFFU;
    s_id_tx_ms = 0U;
    s_id_result = 0U;
    s_bcast_pending = 0U;
    s_dir_swap = 0U;
    s_scan_active = 0U;
    s_scan_done = 0U;
    s_scan_id = 0U;
    s_scan_first = 0U;
    s_scan_last = 0U;
    s_scan_retried = 0U;
    s_scan_count = 0U;
    s_line_len = 0U;
    s_line_active = 0U;

    /* 单总线空闲需要上拉；先启用 MCU 内部上拉 */
    {
        GPIO_InitTypeDef gpio;
        gpio.Pin = Servo_Bus_Rx_Pin;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(Servo_Bus_Rx_GPIO_Port, &gpio);
    }

    servo_set_dir(0U);

    Uart_Rx_Register(&huart4, servo_rx_byte);
    Uart_Rx_Start(&huart4);
}

void Arm_Servo_Update(void)
{
    if (s_pending_slot != SERVO_PENDING_NONE
        && (HAL_GetTick() - s_tx_ms) > ARM_SERVO_TIMEOUT_MS)
    {
        s_err[s_pending_slot]++;
        s_online[s_pending_slot] = 0U;
        s_pending_slot = SERVO_PENDING_NONE;
    }

    if (s_id_pending != 0xFFU
        && (HAL_GetTick() - s_id_tx_ms) > ARM_SERVO_TIMEOUT_MS)
    {
        s_id_result = 2U;
        s_id_pending = 0xFFU;
    }

    if (s_any_pending != 0U
        && (HAL_GetTick() - s_any_tx_ms) > ARM_SERVO_TIMEOUT_MS)
    {
        s_any_pending = 0U;
    }

    if (s_bcast_pending != 0U
        && (HAL_GetTick() - s_id_tx_ms) > ARM_SERVO_TIMEOUT_MS)
    {
        s_id_result = 2U;
        s_bcast_pending = 0U;
    }

    /* 地址扫描状态机：一次只等一个 ID 的应答 */
    if (s_scan_active != 0U)
    {
        if (s_id_result == 0U)
        {
            return;
        }

        if (s_id_result == 1U && s_scan_count < SERVO_SCAN_MAX)
        {
            s_scan_ids[s_scan_count++] = s_scan_id;
        }

        s_id_result = 0U;

        if (s_scan_id >= s_scan_last)
        {
            /* 第一次一个都没找到：交换 TX/RX 开关再试一次 */
            if (s_scan_count == 0U && s_scan_retried == 0U)
            {
                s_scan_retried = 1U;
                s_dir_swap = 1U;
                s_scan_id = s_scan_first;
                s_scan_count = 0U;
                s_id_result = 0U;
                Arm_Servo_RequestId(s_scan_id);
                return;
            }

            if (s_scan_count == 0U)
            {
                s_dir_swap = 0U;
            }

            s_scan_active = 0U;
            s_scan_done = 1U;
            return;
        }

        s_scan_id++;
        Arm_Servo_RequestId(s_scan_id);
    }
}

uint8_t Arm_Servo_IsOnline(uint8_t id)
{
    int8_t slot = servo_slot_of_id(id);

    if (slot < 0)
    {
        return 0U;
    }

    return (s_online[slot] != 0U
            && (HAL_GetTick() - s_last_rx_ms[slot]) < ARM_SERVO_ONLINE_MS) ? 1U : 0U;
}

uint32_t Arm_Servo_GetErrorCount(uint8_t id)
{
    int8_t slot = servo_slot_of_id(id);

    return (slot < 0) ? 0U : s_err[slot];
}

int Arm_Servo_SetId(uint8_t old_id, uint8_t new_id)
{
    char  buf[16];
    char *p = buf;
    int   rc;

    if (new_id > 254U)
    {
        return -1;
    }

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, old_id);
    *p++ = 'P';
    *p++ = 'I';
    *p++ = 'D';
    p = put_u3(p, new_id);
    *p++ = '!';

    s_id_pending = new_id;
    s_id_tx_ms = HAL_GetTick();
    s_id_result = 0U;

    rc = servo_send(buf, (uint16_t)(p - buf));
    if (rc != 0)
    {
        s_id_result = 2U;
        s_id_pending = 0xFFU;
    }

    return rc;
}

void Arm_Servo_RequestId(uint8_t id)
{
    char  buf[12];
    char *p = buf;

    servo_cancel_pending();

    *p++ = '#';
    p = put_u3(p, id);
    *p++ = 'P';
    *p++ = 'I';
    *p++ = 'D';
    *p++ = '!';

    s_id_pending = id;
    s_id_tx_ms = HAL_GetTick();
    s_id_result = 0U;

    (void)servo_send(buf, (uint16_t)(p - buf));
}

uint8_t Arm_Servo_GetIdResult(void)
{
    return s_id_result;
}

void Arm_Servo_RequestBroadcastQuery(void)
{
    char  buf[12];
    char *p = buf;

    servo_cancel_pending();
    s_id_pending = 0xFFU;

    /* 给上一笔回读留出收尾时间，避免旧应答被当成广播结果 */
    HAL_Delay(3U);

    *p++ = '#';
    *p++ = '2';
    *p++ = '5';
    *p++ = '5';
    *p++ = 'P';
    *p++ = 'I';
    *p++ = 'D';
    *p++ = '!';

    s_bcast_pending = 1U;
    s_id_result = 0U;
    s_id_tx_ms = HAL_GetTick();

    (void)servo_send(buf, (uint16_t)(p - buf));
}

void Arm_Servo_ScanStart(uint8_t first, uint8_t last)
{
    if (first > last || last >= SERVO_SCAN_MAX)
    {
        return;
    }

    s_dir_swap = 0U;
    s_scan_active = 1U;
    s_scan_done = 0U;
    s_scan_id = first;
    s_scan_first = first;
    s_scan_last = last;
    s_scan_retried = 0U;
    s_scan_count = 0U;
    s_id_result = 0U;

    Arm_Servo_RequestId(first);
}

uint8_t Arm_Servo_ScanIsActive(void)
{
    return s_scan_active;
}

uint8_t Arm_Servo_ScanIsDone(void)
{
    return s_scan_done;
}

uint8_t Arm_Servo_ScanCount(void)
{
    return s_scan_count;
}

uint8_t Arm_Servo_ScanId(uint8_t index)
{
    if (index >= s_scan_count)
    {
        return 0xFFU;
    }

    return s_scan_ids[index];
}

int Arm_Servo_GetLastId(uint8_t *id)
{
    if (id == NULL || s_id_valid == 0U)
    {
        return -1;
    }

    *id = s_last_id;
    return 0;
}
