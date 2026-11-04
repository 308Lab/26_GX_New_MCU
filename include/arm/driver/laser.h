/**
  ******************************************************************************
  * @file    arm/driver/laser.h
  * @brief   激光 GPIO 驱动。引脚确定前编译为空操作。
  ******************************************************************************
  */

#ifndef ARM_DRIVER_LASER_H
#define ARM_DRIVER_LASER_H

#include <stdint.h>

void Arm_Laser_Init(void);
void Arm_Laser_Set(uint8_t on);
uint8_t Arm_Laser_IsOn(void);
uint8_t Arm_Laser_IsAvailable(void);

#endif /* ARM_DRIVER_LASER_H */
