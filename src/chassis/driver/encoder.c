/**
  ******************************************************************************
  * @file    chassis/driver/encoder.c
  ******************************************************************************
  */

#include "chassis/driver/encoder.h"
#include "chassis/chassis_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* 硬件通道按 A/B 排列 */
static TIM_HandleTypeDef *const s_htim[2] = { &htim1, &htim8 };

static const uint8_t s_logic_to_hw[CHASSIS_ENC_NUM] =
{
#if CHASSIS_ENC_A_IS_LEFT
    (uint8_t)0, (uint8_t)1
#else
    (uint8_t)1, (uint8_t)0
#endif
};

static const uint8_t s_invert[CHASSIS_ENC_NUM] =
{
    (uint8_t)CHASSIS_ENC_L_INVERT,
    (uint8_t)CHASSIS_ENC_R_INVERT
};

static uint16_t s_last_hw[2];
static int32_t  s_total[CHASSIS_ENC_NUM];

static void encoder_apply_filter(TIM_TypeDef *tim)
{
    if (CHASSIS_ENC_FILTER_ICF > 0U)
    {
        MODIFY_REG(tim->CCMR1,
                   TIM_CCMR1_IC1F | TIM_CCMR1_IC2F,
                   ((uint32_t)CHASSIS_ENC_FILTER_ICF << TIM_CCMR1_IC1F_Pos)
                 | ((uint32_t)CHASSIS_ENC_FILTER_ICF << TIM_CCMR1_IC2F_Pos));
    }
}

void Chassis_Encoder_Init(void)
{
    encoder_apply_filter(htim1.Instance);
    encoder_apply_filter(htim8.Instance);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);

    s_last_hw[0] = 0U;
    s_last_hw[1] = 0U;
    s_total[CHASSIS_ENC_LEFT] = 0;
    s_total[CHASSIS_ENC_RIGHT] = 0;
}

int16_t Chassis_Encoder_ReadDelta(Chassis_Enc_Id_e id)
{
    TIM_HandleTypeDef *htim;
    uint32_t hw;
    uint16_t now;
    int16_t raw_delta;
    int16_t delta;

    if ((uint32_t)id >= CHASSIS_ENC_NUM)
    {
        return 0;
    }

    hw = s_logic_to_hw[id];
    htim = s_htim[hw];

    now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    raw_delta = (int16_t)(uint16_t)(now - s_last_hw[hw]);
    s_last_hw[hw] = now;

    delta = (s_invert[id] != 0U) ? (int16_t)(-raw_delta) : raw_delta;
    s_total[id] += (int32_t)delta;

    return delta;
}

int32_t Chassis_Encoder_GetTotal(Chassis_Enc_Id_e id)
{
    if ((uint32_t)id >= CHASSIS_ENC_NUM)
    {
        return 0;
    }
    return s_total[id];
}

void Chassis_Encoder_ResetTotal(Chassis_Enc_Id_e id)
{
    if ((uint32_t)id >= CHASSIS_ENC_NUM)
    {
        return;
    }
    s_total[id] = 0;
}
