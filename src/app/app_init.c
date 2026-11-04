/**
  ******************************************************************************
  * @file    app/app_init.c
  ******************************************************************************
  */

#include "app/app_init.h"
#include "app/mission.h"
#include "app/vision_link.h"
#include "arm/arm.h"
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
}
