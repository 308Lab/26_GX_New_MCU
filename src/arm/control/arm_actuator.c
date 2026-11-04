/**
  ******************************************************************************
  * @file    arm/control/arm_actuator.c
  ******************************************************************************
  */

#include "arm/control/arm_actuator.h"
#include "arm/driver/gripper.h"
#include "arm/driver/laser.h"

static Arm_Gripper_State_e s_gripper;
static uint8_t s_laser_on;

void Arm_Actuator_Init(void)
{
    /* 夹爪驱动上电初始化在关闭位（135°），状态保持一致 */
    s_gripper = ARM_GRIPPER_CLOSE;
    s_laser_on = 0U;
}

int Arm_Actuator_Gripper(Arm_Gripper_State_e state)
{
    switch (state)
    {
    case ARM_GRIPPER_OPEN:
        Arm_Gripper_Open();
        break;

    case ARM_GRIPPER_CLOSE:
        Arm_Gripper_Close();
        break;

    case ARM_GRIPPER_GRASP:
        Arm_Gripper_Grasp();
        break;

    case ARM_GRIPPER_HOLD:
        break;

    default:
        return -1;
    }

    s_gripper = state;
    return 0;
}

int Arm_Actuator_Laser(uint8_t on)
{
    Arm_Laser_Set(on);
    s_laser_on = (on != 0U) ? 1U : 0U;
    return 0;
}

Arm_Gripper_State_e Arm_Actuator_GetGripper(void)
{
    return s_gripper;
}

uint8_t Arm_Actuator_LaserIsOn(void)
{
    return s_laser_on;
}
