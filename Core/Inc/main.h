/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Laser_Pin GPIO_PIN_1
#define Laser_GPIO_Port GPIOC
#define Gyro_Tx_Pin GPIO_PIN_2
#define Gyro_Tx_GPIO_Port GPIOA
#define Gyro_Rx_Pin GPIO_PIN_3
#define Gyro_Rx_GPIO_Port GPIOA
#define MOTOR_PWM_A_Pin GPIO_PIN_0
#define MOTOR_PWM_A_GPIO_Port GPIOB
#define MOTOR_PWM_B_Pin GPIO_PIN_1
#define MOTOR_PWM_B_GPIO_Port GPIOB
#define Key_Row_2_Pin GPIO_PIN_2
#define Key_Row_2_GPIO_Port GPIOB
#define Key_Row_3_Pin GPIO_PIN_7
#define Key_Row_3_GPIO_Port GPIOE
#define ENC_MA_A_Pin GPIO_PIN_9
#define ENC_MA_A_GPIO_Port GPIOE
#define Key_Row_1_Pin GPIO_PIN_10
#define Key_Row_1_GPIO_Port GPIOE
#define ENC_MA_B_Pin GPIO_PIN_11
#define ENC_MA_B_GPIO_Port GPIOE
#define Key_Col_1_Pin GPIO_PIN_13
#define Key_Col_1_GPIO_Port GPIOE
#define Key_Col_2_Pin GPIO_PIN_14
#define Key_Col_2_GPIO_Port GPIOE
#define Key_Col_3_Pin GPIO_PIN_15
#define Key_Col_3_GPIO_Port GPIOE
#define Host_Tx_Pin GPIO_PIN_10
#define Host_Tx_GPIO_Port GPIOB
#define Host_Rx_Pin GPIO_PIN_11
#define Host_Rx_GPIO_Port GPIOB
#define MOTOR_BIN_2_Pin GPIO_PIN_12
#define MOTOR_BIN_2_GPIO_Port GPIOB
#define MOTOR_BIN_1_Pin GPIO_PIN_13
#define MOTOR_BIN_1_GPIO_Port GPIOB
#define MOTOR_AIN_1_Pin GPIO_PIN_8
#define MOTOR_AIN_1_GPIO_Port GPIOD
#define MOTOR_AIN_2_Pin GPIO_PIN_9
#define MOTOR_AIN_2_GPIO_Port GPIOD
#define Servo_PWM_Pin GPIO_PIN_15
#define Servo_PWM_GPIO_Port GPIOD
#define ENC_MB_A_Pin GPIO_PIN_6
#define ENC_MB_A_GPIO_Port GPIOC
#define ENC_MB_B_Pin GPIO_PIN_7
#define ENC_MB_B_GPIO_Port GPIOC
#define Bsvo_TxSwitch_Pin GPIO_PIN_9
#define Bsvo_TxSwitch_GPIO_Port GPIOC
#define Step_Tx_Pin GPIO_PIN_9
#define Step_Tx_GPIO_Port GPIOA
#define Step_Rx_Pin GPIO_PIN_10
#define Step_Rx_GPIO_Port GPIOA
#define Bsvo_RxSwitch_Pin GPIO_PIN_11
#define Bsvo_RxSwitch_GPIO_Port GPIOA
#define Servo_Bus_Tx_Pin GPIO_PIN_10
#define Servo_Bus_Tx_GPIO_Port GPIOC
#define Servo_Bus_Rx_Pin GPIO_PIN_11
#define Servo_Bus_Rx_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
