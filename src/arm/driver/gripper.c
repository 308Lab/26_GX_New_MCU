/**
  ******************************************************************************
  * @file    arm/driver/gripper.c
  * @brief   夹爪 PWM（TIM4 CH4，PD15，50 Hz）。1 tick = 1 us。
  ******************************************************************************
  */

#include "arm/driver/gripper.h"
#include "arm/arm_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

static uint16_t s_pulse_us = ARM_GRIPPER_CLOSE_US;

void Arm_Gripper_Init(void)
{
    /* 初始为关闭位（135°），先写入脉宽再启动 PWM，避免上电瞬间输出 0% */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ARM_GRIPPER_CLOSE_US);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    s_pulse_us = ARM_GRIPPER_CLOSE_US;
}

void Arm_Gripper_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < 500U)
    {
        pulse_us = 500U;
    }
    if (pulse_us > 2500U)
    {
        pulse_us = 2500U;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_us);
    s_pulse_us = pulse_us;
}

void Arm_Gripper_Open(void)
{
    Arm_Gripper_SetPulseUs(ARM_GRIPPER_OPEN_US);
}

void Arm_Gripper_Close(void)
{
    Arm_Gripper_SetPulseUs(ARM_GRIPPER_CLOSE_US);
}

void Arm_Gripper_Grasp(void)
{
    Arm_Gripper_SetPulseUs(ARM_GRIPPER_GRASP_US);
}

uint16_t Arm_Gripper_GetPulseUs(void)
{
    return s_pulse_us;
}
