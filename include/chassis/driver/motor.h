/**
  ******************************************************************************
  * @file    chassis/driver/motor.h
  * @brief   底盘电机驱动：方向 GPIO + TIM3 PWM。
  *
  * 逻辑正 pwm 表示车体前进，负值后退，0 为自由滑行。
  ******************************************************************************
  */

#ifndef CHASSIS_DRIVER_MOTOR_H
#define CHASSIS_DRIVER_MOTOR_H

#include <stdint.h>

typedef enum
{
    CHASSIS_MOTOR_LEFT = 0,
    CHASSIS_MOTOR_RIGHT = 1,
    CHASSIS_MOTOR_NUM
} Chassis_Motor_Id_e;

/* 启动 TIM3 CH3/CH4 并把两路置为零占空比滑行态 */
void Chassis_Motor_Init(void);

/* pwm: -CHASSIS_PWM_MAX ~ +CHASSIS_PWM_MAX */
void Chassis_Motor_SetOutput(Chassis_Motor_Id_e id, int16_t pwm);

/* 短路制动：IN1 = IN2 = 1，PWM = 0 */
void Chassis_Motor_Brake(Chassis_Motor_Id_e id);

/* 自由滑行：IN1 = IN2 = 0，PWM = 0 */
void Chassis_Motor_Coast(Chassis_Motor_Id_e id);

/* brake = 1 短路制动，brake = 0 自由滑行 */
void Chassis_Motor_StopAll(uint8_t brake);

int16_t Chassis_Motor_GetOutput(Chassis_Motor_Id_e id);

#endif /* CHASSIS_DRIVER_MOTOR_H */
