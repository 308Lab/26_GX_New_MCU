/**
  ******************************************************************************
  * @file    app/app_init.c
  ******************************************************************************
  */

#include "app/app_init.h"
#include "app/mission.h"
#include "app/vision_link.h"
#include "arm/arm.h"
#include "arm/driver/servo_bus.h"
#include "arm/driver/step_uart.h"
#include "chassis/chassis.h"
#include "common/keypad.h"
#include "common/oled.h"

void App_Init(void)
{
    Oled_Init();
    Keypad_Init();
    Chassis_Init();
    (void)Arm_Init();

    /* 最后注册 USART3：uart_rx 共 4 槽，huart1/2/4 已先占用 */
    Vision_Link_Init();
    Mission_Init();
}

void App_InitRtos(void)
{
    Chassis_InitRtos();

    /* 审计 H10：两条命令总线（UART4 半双工舵机 / USART1 步进）各一把发送锁。
       ArmTask 与 MainTask02 都会下发，必须先有锁再放任务起来。 */
    Arm_Servo_InitRtos();
    Arm_Step_InitRtos();
}
