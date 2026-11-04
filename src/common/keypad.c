/**
  ******************************************************************************
  * @file    keypad.c
  * @brief   3x3 矩阵键盘非阻塞扫描。
  *
  * 相位序列（每拍一步）：
  *   拍1 DRIVE0 : 行0 拉低，其余拉高
  *   拍2 READ0  : 读三列记录行0；行1 拉低
  *   拍3 READ1  : 读三列记录行1；行2 拉低
  *   拍4 READ2  : 读三列记录行2；三行全部拉高；评估本轮
  *
  * 关键：拉低与读取分处相邻两拍，中间相隔 5ms，无需任何延时等待。
  ******************************************************************************
  */

#include "common/keypad.h"
#include "main.h"

#define KEYPAD_PIN_NONE   (0xFFU)

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} Keypad_Hw_t;

static const Keypad_Hw_t s_rows[3] =
{
    { Key_Row_1_GPIO_Port, Key_Row_1_Pin },
    { Key_Row_2_GPIO_Port, Key_Row_2_Pin },
    { Key_Row_3_GPIO_Port, Key_Row_3_Pin }
};

static const Keypad_Hw_t s_cols[3] =
{
    { Key_Col_1_GPIO_Port, Key_Col_1_Pin },
    { Key_Col_2_GPIO_Port, Key_Col_2_Pin },
    { Key_Col_3_GPIO_Port, Key_Col_3_Pin }
};

/* 行 1..3 × 列 1..3 */
static const uint8_t s_keymap[3][3] =
{
    { '7', '8', '9' },
    { '4', '5', '6' },
    { '1', '2', '3' }
};

typedef enum
{
    KP_PHASE_DRIVE0 = 0,
    KP_PHASE_READ0,
    KP_PHASE_READ1,
    KP_PHASE_READ2
} Keypad_Phase_e;

static Keypad_Phase_e s_phase;
static uint8_t s_row_hit;
static uint8_t s_col_hit;

static uint8_t s_last_raw;
static uint8_t s_stable;
static uint8_t s_stable_cnt;
static volatile uint8_t s_event;

static void keypad_drive_row(uint8_t active)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        HAL_GPIO_WritePin(s_rows[i].port, s_rows[i].pin,
                          (i == active) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/* 返回命中的列号 0..2，无命中返回 KEYPAD_PIN_NONE */
static uint8_t keypad_read_cols(void)
{
    uint8_t c;

    for (c = 0U; c < 3U; c++)
    {
        if (HAL_GPIO_ReadPin(s_cols[c].port, s_cols[c].pin) == GPIO_PIN_RESET)
        {
            return c;
        }
    }

    return KEYPAD_PIN_NONE;
}

static void keypad_debounce(uint8_t raw)
{
    if (raw == s_last_raw)
    {
        if (s_stable_cnt < 0xFFU)
        {
            s_stable_cnt++;
        }
    }
    else
    {
        s_last_raw = raw;
        s_stable_cnt = 1U;   /* 本次已算一个采样 */
    }

    if (s_stable_cnt >= KEYPAD_DEBOUNCE_ROUNDS && raw != s_stable)
    {
        if (s_stable == KEYPAD_KEY_NONE && raw != KEYPAD_KEY_NONE)
        {
            s_event = raw;
        }
        s_stable = raw;
    }
}

void Keypad_Init(void)
{
    keypad_drive_row(KEYPAD_PIN_NONE);   /* 三行全部拉高 */

    s_phase = KP_PHASE_DRIVE0;
    s_row_hit = KEYPAD_PIN_NONE;
    s_col_hit = 0U;

    s_last_raw = KEYPAD_KEY_NONE;
    s_stable = KEYPAD_KEY_NONE;
    s_stable_cnt = 0U;
    s_event = KEYPAD_KEY_NONE;
}

void Keypad_Scan(void)
{
    uint8_t col;
    uint8_t raw;

    switch (s_phase)
    {
    case KP_PHASE_DRIVE0:
        keypad_drive_row(0U);
        s_phase = KP_PHASE_READ0;
        break;

    case KP_PHASE_READ0:
        col = keypad_read_cols();
        if (col != KEYPAD_PIN_NONE && s_row_hit == KEYPAD_PIN_NONE)
        {
            s_row_hit = 0U;
            s_col_hit = col;
        }
        keypad_drive_row(1U);
        s_phase = KP_PHASE_READ1;
        break;

    case KP_PHASE_READ1:
        col = keypad_read_cols();
        if (col != KEYPAD_PIN_NONE && s_row_hit == KEYPAD_PIN_NONE)
        {
            s_row_hit = 1U;
            s_col_hit = col;
        }
        keypad_drive_row(2U);
        s_phase = KP_PHASE_READ2;
        break;

    case KP_PHASE_READ2:
    default:
        col = keypad_read_cols();
        if (col != KEYPAD_PIN_NONE && s_row_hit == KEYPAD_PIN_NONE)
        {
            s_row_hit = 2U;
            s_col_hit = col;
        }

        keypad_drive_row(KEYPAD_PIN_NONE);   /* 恢复全高 */

        if (s_row_hit != KEYPAD_PIN_NONE)
        {
            raw = s_keymap[s_row_hit][s_col_hit];
        }
        else
        {
            raw = KEYPAD_KEY_NONE;
        }

        keypad_debounce(raw);

        s_row_hit = KEYPAD_PIN_NONE;
        s_phase = KP_PHASE_DRIVE0;
        break;
    }
}

uint8_t Keypad_GetEvent(void)
{
    uint8_t key = s_event;

    s_event = KEYPAD_KEY_NONE;
    return key;
}

uint8_t Keypad_GetState(void)
{
    return s_stable;
}
