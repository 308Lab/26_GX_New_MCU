/**
  ******************************************************************************
  * @file    arm/perception/joint_fb.c
  * @brief   关节反馈：轮询回读、零位/方向换算、在线状态。
  *
  * 只依赖 driver 层，不包含 control/planning。
  ******************************************************************************
  */

#include "arm/perception/joint_fb.h"
#include "arm/arm_types.h"
#include "arm/arm_config.h"
#include "arm/driver/step_uart.h"
#include "arm/driver/servo_bus.h"
#include "common/math_util.h"
#include "main.h"

static uint32_t s_next_ms;
static uint8_t  s_axis;
static uint8_t  s_poll_active;

/* J1 无绝对零位：上电前手动对正，软件从 0° 开始累计相对移动。 */
static float   s_j1_pos_deg;
static uint8_t s_j1_valid;

static int servo_axis_id(uint8_t axis, uint8_t *id)
{
    if (id == NULL)
    {
        return -1;
    }

    switch (axis)
    {
    case ARM_AXIS_J2: *id = (uint8_t)ARM_SERVO_ID_J2; return 0;
    case ARM_AXIS_J3: *id = (uint8_t)ARM_SERVO_ID_J3; return 0;
    case ARM_AXIS_J4: *id = (uint8_t)ARM_SERVO_ID_J4; return 0;
    default: return -1;
    }
}

static float servo_zero_pwm(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_ZERO_PWM;
    case ARM_AXIS_J3: return ARM_SERVO_J3_ZERO_PWM;
    case ARM_AXIS_J4: return ARM_SERVO_J4_ZERO_PWM;
    default: return 1500.0f;
    }
}

static float servo_dir(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_DIR;
    case ARM_AXIS_J3: return ARM_SERVO_J3_DIR;
    case ARM_AXIS_J4: return ARM_SERVO_J4_DIR;
    default: return 1.0f;
    }
}

static float servo_pwm_per_deg(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_PWM_PER_DEG;
    case ARM_AXIS_J3: return ARM_SERVO_J3_PWM_PER_DEG;
    case ARM_AXIS_J4: return ARM_SERVO_J4_PWM_PER_DEG;
    default: return 2000.0f / ARM_SERVO_DEG_RANGE;
    }
}

void Arm_Fb_Init(void)
{
    s_next_ms = 0U;
    s_axis = ARM_AXIS_J1;
    s_poll_active = 0U;

    s_j1_pos_deg = 0.0f;
    s_j1_valid = 1U;
}

void Arm_Fb_AddJ1Delta(float delta_deg)
{
    s_j1_pos_deg = Math_Wrap180(s_j1_pos_deg + delta_deg);
}

void Arm_Fb_RequestAll(void)
{
    /* 已经在一轮回读中就不要重置，否则动作重试会永远停在 J1 */
    if (s_poll_active != 0U)
    {
        return;
    }

    s_poll_active = 1U;
    s_axis = ARM_AXIS_J1;
    s_next_ms = HAL_GetTick();
}

void Arm_Fb_Update(void)
{
    uint8_t id;
    uint32_t now = HAL_GetTick();

    /* 默认不主动回读；只有 RequestAll 触发一轮（4 轴，每轴 50 ms） */
    if (s_poll_active == 0U)
    {
        return;
    }

    if ((int32_t)(now - s_next_ms) < 0)
    {
        return;
    }

    switch (s_axis)
    {
    case ARM_AXIS_J1:
        Arm_Step_RequestPosition();
        break;

    case ARM_AXIS_J2:
    case ARM_AXIS_J3:
    case ARM_AXIS_J4:
        if (servo_axis_id(s_axis, &id) == 0)
        {
            Arm_Servo_RequestPosition(id);
        }
        break;

    default:
        break;
    }

    s_axis++;
    if (s_axis >= ARM_AXIS_NUM)
    {
        s_poll_active = 0U;
        return;
    }

    s_next_ms = now + ARM_FB_FAST_POLL_MS;
}

int Arm_Fb_GetJointsMask(float q_deg[ARM_FB_AXIS_NUM], uint8_t *mask)
{
    uint8_t i;
    uint8_t valid = 0U;
    uint16_t pwm;
    uint8_t id;

    if (q_deg == NULL || mask == NULL)
    {
        return 0;
    }

    *mask = 0U;

    for (i = 0U; i < ARM_FB_AXIS_NUM; i++)
    {
        q_deg[i] = 0.0f;
    }

    if (s_j1_valid != 0U && Arm_Step_IsOnline() != 0U)
    {
        q_deg[ARM_AXIS_J1] = s_j1_pos_deg;
        *mask |= (uint8_t)(1U << ARM_AXIS_J1);
        valid++;
    }

    /* 回读是逐轴先后进行的，不能用“当前在线”卡住整组：
       只要该轴曾经成功回读过（缓存有效）就返回其位置。 */
    for (i = ARM_AXIS_J2; i < ARM_AXIS_NUM; i++)
    {
        if (servo_axis_id(i, &id) == 0
            && Arm_Servo_GetPositionPwm(id, &pwm) == 0)
        {
            q_deg[i] = ((float)pwm - servo_zero_pwm(i))
                     / servo_pwm_per_deg(i)
                     * servo_dir(i);
            *mask |= (uint8_t)(1U << i);
            valid++;
        }
    }

    return (valid > 0U) ? 1 : 0;
}

int Arm_Fb_GetJoints(float q_deg[ARM_FB_AXIS_NUM])
{
    uint8_t mask;

    return Arm_Fb_GetJointsMask(q_deg, &mask);
}

int Arm_Fb_IsOnline(uint8_t axis)
{
    uint8_t id;

    if (axis == ARM_AXIS_J1)
    {
        return (Arm_Step_IsOnline() != 0U) ? 1 : 0;
    }

    if (servo_axis_id(axis, &id) == 0)
    {
        return (Arm_Servo_IsOnline(id) != 0U) ? 1 : 0;
    }

    return 0;
}

uint32_t Arm_Fb_GetErrorCount(uint8_t axis)
{
    uint8_t id;

    if (axis == ARM_AXIS_J1)
    {
        return Arm_Step_GetErrorCount();
    }

    if (servo_axis_id(axis, &id) == 0)
    {
        return Arm_Servo_GetErrorCount(id);
    }

    return 0U;
}

int Arm_Fb_GetServoPwm(uint8_t axis, uint16_t *pwm)
{
    uint8_t id;

    if (servo_axis_id(axis, &id) == 0)
    {
        return Arm_Servo_GetPositionPwm(id, pwm);
    }

    return -1;
}

int Arm_Fb_GetStepPosition(float *deg)
{
    if (deg == NULL || s_j1_valid == 0U)
    {
        return -1;
    }

    *deg = s_j1_pos_deg;
    return 0;
}
