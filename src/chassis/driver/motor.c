/**
  ******************************************************************************
  * @file    chassis/driver/motor.c
  ******************************************************************************
  */

#include "chassis/driver/motor.h"
#include "chassis/chassis_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

typedef struct
{
    uint32_t      channel;
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
} Motor_Hw_t;

/* 硬件通道按 A/B 排列，逻辑左右通过 s_logic_to_hw 映射 */
static const Motor_Hw_t s_hw[2] =
{
    { TIM_CHANNEL_3, MOTOR_AIN_1_GPIO_Port, MOTOR_AIN_1_Pin,
                     MOTOR_AIN_2_GPIO_Port, MOTOR_AIN_2_Pin },
    { TIM_CHANNEL_4, MOTOR_BIN_1_GPIO_Port, MOTOR_BIN_1_Pin,
                     MOTOR_BIN_2_GPIO_Port, MOTOR_BIN_2_Pin }
};

static const uint8_t s_logic_to_hw[CHASSIS_MOTOR_NUM] =
{
#if CHASSIS_MOTOR_A_IS_LEFT
    (uint8_t)0, (uint8_t)1
#else
    (uint8_t)1, (uint8_t)0
#endif
};

static const uint8_t s_invert[CHASSIS_MOTOR_NUM] =
{
    (uint8_t)CHASSIS_MOTOR_L_INVERT,
    (uint8_t)CHASSIS_MOTOR_R_INVERT
};

/* 左右轮静摩擦死区不同，分别补偿 */
static const uint16_t s_min_move[CHASSIS_MOTOR_NUM] =
{
    (uint16_t)CHASSIS_PWM_MIN_MOVE_L,
    (uint16_t)CHASSIS_PWM_MIN_MOVE_R
};

static int16_t s_out[CHASSIS_MOTOR_NUM];

static void motor_set_pins(const Motor_Hw_t *hw, uint8_t in1, uint8_t in2)
{
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin,
                      in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin,
                      in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Chassis_Motor_Init(void)
{
    /* 先清零比较值：MX_TIM3_Init 的 Pulse=20 会在 Start 瞬间输出 2% 占空比 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    Chassis_Motor_StopAll(0U);
}

void Chassis_Motor_SetOutput(Chassis_Motor_Id_e id, int16_t pwm)
{
    const Motor_Hw_t *hw;
    uint16_t duty;
    uint8_t forward;

    if ((uint32_t)id >= CHASSIS_MOTOR_NUM)
    {
        return;
    }

    if (pwm > CHASSIS_PWM_MAX)
    {
        pwm = CHASSIS_PWM_MAX;
    }
    if (pwm < -CHASSIS_PWM_MAX)
    {
        pwm = -CHASSIS_PWM_MAX;
    }

    forward = (pwm >= 0) ? 1U : 0U;
    if (s_invert[id] != 0U)
    {
        forward = (forward != 0U) ? 0U : 1U;
    }

    duty = (uint16_t)((pwm >= 0) ? pwm : -pwm);
    if (duty > 0U && duty < s_min_move[id])
    {
        duty = s_min_move[id];
    }

    hw = &s_hw[s_logic_to_hw[id]];

    if (duty == 0U)
    {
        motor_set_pins(hw, 0U, 0U);   /* 自由滑行 */
    }
    else if (forward != 0U)
    {
        motor_set_pins(hw, 1U, 0U);
    }
    else
    {
        motor_set_pins(hw, 0U, 1U);
    }

    __HAL_TIM_SET_COMPARE(&htim3, hw->channel, duty);
    s_out[id] = pwm;
}

void Chassis_Motor_Brake(Chassis_Motor_Id_e id)
{
    const Motor_Hw_t *hw;

    if ((uint32_t)id >= CHASSIS_MOTOR_NUM)
    {
        return;
    }

    hw = &s_hw[s_logic_to_hw[id]];
    __HAL_TIM_SET_COMPARE(&htim3, hw->channel, 0U);
    motor_set_pins(hw, 1U, 1U);
    s_out[id] = 0;
}

void Chassis_Motor_Coast(Chassis_Motor_Id_e id)
{
    const Motor_Hw_t *hw;

    if ((uint32_t)id >= CHASSIS_MOTOR_NUM)
    {
        return;
    }

    hw = &s_hw[s_logic_to_hw[id]];
    __HAL_TIM_SET_COMPARE(&htim3, hw->channel, 0U);
    motor_set_pins(hw, 0U, 0U);
    s_out[id] = 0;
}

void Chassis_Motor_StopAll(uint8_t brake)
{
    Chassis_Motor_Id_e id;

    for (id = CHASSIS_MOTOR_LEFT; id < CHASSIS_MOTOR_NUM; id++)
    {
        if (brake != 0U)
        {
            Chassis_Motor_Brake(id);
        }
        else
        {
            Chassis_Motor_Coast(id);
        }
    }
}

int16_t Chassis_Motor_GetOutput(Chassis_Motor_Id_e id)
{
    if ((uint32_t)id >= CHASSIS_MOTOR_NUM)
    {
        return 0;
    }
    return s_out[id];
}
