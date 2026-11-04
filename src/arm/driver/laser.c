/**
  ******************************************************************************
  * @file    arm/driver/laser.c
  * @brief   激光 GPIO 驱动。引脚确定前所有操作为空操作。
  ******************************************************************************
  */

#include "arm/driver/laser.h"
#include "arm/arm_config.h"

#if ARM_LASER_ENABLE
#include "main.h"

#ifndef ARM_LASER_GPIO_Port
#error "ARM_LASER_ENABLE=1 但未定义 ARM_LASER_GPIO_Port/ARM_LASER_Pin"
#endif
#endif

static uint8_t s_on;

void Arm_Laser_Init(void)
{
    s_on = 0U;

#if ARM_LASER_ENABLE
    HAL_GPIO_WritePin(ARM_LASER_GPIO_Port, ARM_LASER_Pin, GPIO_PIN_RESET);
#endif
}

void Arm_Laser_Set(uint8_t on)
{
    s_on = (on != 0U) ? 1U : 0U;

#if ARM_LASER_ENABLE
    HAL_GPIO_WritePin(ARM_LASER_GPIO_Port, ARM_LASER_Pin,
                      s_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint8_t Arm_Laser_IsOn(void)
{
    return s_on;
}

uint8_t Arm_Laser_IsAvailable(void)
{
    return (ARM_LASER_ENABLE != 0) ? 1U : 0U;
}
