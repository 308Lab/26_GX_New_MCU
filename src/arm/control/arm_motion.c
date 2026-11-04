/**
  ******************************************************************************
  * @file    arm/control/arm_motion.c
  * @brief   PTP 多轴时间对齐：舵机同步帧 + 步进位置模式，同一拍下发。
  ******************************************************************************
  */

#include "arm/control/arm_motion.h"
#include "arm/control/arm_limit.h"
#include "arm/arm_config.h"
#include "arm/driver/step_uart.h"
#include "arm/driver/servo_bus.h"
#include "arm/perception/joint_fb.h"
#include "common/math_util.h"
#include "main.h"

#include <math.h>

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_BUSY,
    MOTION_DONE,
    MOTION_FAULT
} Motion_State_e;

static Motion_State_e s_state;
static Arm_Status_e   s_fault;

static float    s_target[4];
static uint32_t s_start_ms;
static uint32_t s_motion_time_ms;
static uint32_t s_timeout_ms;
static uint32_t s_last_fb_req_ms;
static uint8_t  s_settle_count;

static const uint8_t s_servo_ids[3] =
{
    (uint8_t)ARM_SERVO_ID_J2,
    (uint8_t)ARM_SERVO_ID_J3,
    (uint8_t)ARM_SERVO_ID_J4
};

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

static void motion_fault(Arm_Status_e err)
{
    s_fault = err;
    s_state = MOTION_FAULT;
    (void)Arm_Step_StopNow();
    Arm_Servo_StopAll();
}

int Arm_Motion_Init(void)
{
    s_state = MOTION_IDLE;
    s_fault = ARM_OK;
    s_settle_count = 0U;
    s_start_ms = 0U;
    s_motion_time_ms = 0U;
    s_timeout_ms = 0U;
    s_last_fb_req_ms = 0U;

    return 0;
}

Arm_Status_e Arm_Motion_StartJoint(const float q_target[4], float min_time_s)
{
    float    q_now[4];
    float    d[4];
    float    t_servo;
    float    t_step;
    float    rpm_f;
    uint8_t  mask;
    uint8_t  i;
    int32_t  pulses;
    uint16_t rpm;
    uint16_t pwm[3];
    uint16_t servo_time_ms;

    if (q_target == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (s_state == MOTION_BUSY)
    {
        return ARM_BUSY;
    }

    if (Arm_Limit_CheckJoints(q_target) != 0)
    {
        return ARM_ERR_LIMIT;
    }

    /* J1 无绝对零位，且部分驱动固件不响应 0x36 位置回读：
       J1 用软件累计位置，只要求 J2/J3/J4 三轴回读就绪。 */
    if (Arm_Fb_GetJointsMask(q_now, &mask) == 0 || (mask & 0x0EU) != 0x0EU)
    {
        /* 反馈未就绪：发起一轮回读，调用方下一拍重试 */
        Arm_Fb_RequestAll();
        return ARM_BUSY;
    }

    /* 用软件累计的 J1 位置覆盖（离线时 GetJointsMask 会给 0） */
    if (Arm_Fb_GetStepPosition(&q_now[ARM_AXIS_J1]) != 0)
    {
        q_now[ARM_AXIS_J1] = 0.0f;
    }

    d[ARM_AXIS_J1] = Math_Wrap180(q_target[ARM_AXIS_J1] - q_now[ARM_AXIS_J1]);
    for (i = ARM_AXIS_J2; i < ARM_AXIS_NUM; i++)
    {
        d[i] = q_target[i] - q_now[i];
    }

    if (fabsf(d[0]) < 0.05f
        && fabsf(d[1]) < 0.05f
        && fabsf(d[2]) < 0.05f
        && fabsf(d[3]) < 0.05f)
    {
        for (i = 0U; i < ARM_AXIS_NUM; i++)
        {
            s_target[i] = q_target[i];
        }
        s_state = MOTION_DONE;
        s_fault = ARM_OK;
        return ARM_OK;
    }

    t_servo = (min_time_s > ARM_PTP_MIN_TIME_S)
            ? min_time_s : ARM_PTP_MIN_TIME_S;

    for (i = ARM_AXIS_J2; i < ARM_AXIS_NUM; i++)
    {
        float t = fabsf(d[i]) / ARM_PTP_SERVO_W_MAX_DEGPS;
        if (t > t_servo)
        {
            t_servo = t;
        }
    }

    pulses = (int32_t)lroundf(d[ARM_AXIS_J1] * ARM_STEP_PULSES_PER_REV
                              * ARM_STEP_GEAR_RATIO / 360.0f);
    rpm = 0U;
    t_step = 0.0f;

    if (pulses != 0)
    {
        rpm_f = fabsf(d[ARM_AXIS_J1]) * ARM_STEP_GEAR_RATIO
              / (6.0f * t_servo);
        if (rpm_f < (float)ARM_STEP_SPEED_MIN_RPM)
        {
            rpm_f = (float)ARM_STEP_SPEED_MIN_RPM;
        }
        if (rpm_f > (float)ARM_STEP_SPEED_MAX_RPM)
        {
            rpm_f = (float)ARM_STEP_SPEED_MAX_RPM;
        }

        rpm = (uint16_t)lroundf(rpm_f);
        t_step = fabsf(d[ARM_AXIS_J1]) * ARM_STEP_GEAR_RATIO
               / (6.0f * (float)rpm);

        if (t_step > t_servo)
        {
            t_servo = t_step;
        }
    }

    if (t_servo > 9.0f)
    {
        return ARM_ERR_PARAM;
    }

    servo_time_ms = (uint16_t)lroundf(t_servo * 1000.0f);

    for (i = 0U; i < 3U; i++)
    {
        uint8_t axis = (uint8_t)(ARM_AXIS_J2 + i);
        float   f = servo_zero_pwm(axis)
                  + q_target[axis] * servo_pwm_per_deg(axis)
                    * servo_dir(axis);

        if (f < (float)ARM_SERVO_PWM_MIN || f > (float)ARM_SERVO_PWM_MAX)
        {
            return ARM_ERR_LIMIT;
        }

        pwm[i] = (uint16_t)lroundf(f);
    }

    s_state = MOTION_BUSY;
    s_fault = ARM_OK;
    s_settle_count = 0U;

    if (Arm_Servo_SetPwmSync(s_servo_ids, pwm, 3U, servo_time_ms) != 0)
    {
        motion_fault(ARM_ERR_COMM);
        return ARM_ERR_COMM;
    }

    if (pulses != 0
        && Arm_Step_MoveRelativePulses(pulses, rpm) != 0)
    {
        motion_fault(ARM_ERR_COMM);
        return ARM_ERR_COMM;
    }

    if (pulses != 0)
    {
        Arm_Fb_AddJ1Delta(d[ARM_AXIS_J1]);
    }

    for (i = 0U; i < ARM_AXIS_NUM; i++)
    {
        s_target[i] = q_target[i];
    }

    s_start_ms = HAL_GetTick();
    s_motion_time_ms = (uint32_t)lroundf(t_servo * 1000.0f);
    s_timeout_ms = s_motion_time_ms + ARM_PTP_TIMEOUT_MARGIN_MS;
    s_last_fb_req_ms = s_start_ms;

    Arm_Fb_RequestAll();

    return ARM_OK;
}

void Arm_Motion_Update(void)
{
    float   q[4];
    uint8_t mask;
    uint8_t ok;
    uint8_t i;
    uint32_t elapsed;

    if (s_state != MOTION_BUSY)
    {
        return;
    }

    elapsed = HAL_GetTick() - s_start_ms;

    if (elapsed > s_timeout_ms)
    {
        motion_fault(ARM_ERR_TIMEOUT);
        return;
    }

    /* 等运动时间过去后再看回读，避免旧数据误判到位 */
    if (elapsed < s_motion_time_ms)
    {
        return;
    }

    /* 到位阶段每 250 ms 刷新一轮回读 */
    if ((HAL_GetTick() - s_last_fb_req_ms) > 250U)
    {
        s_last_fb_req_ms = HAL_GetTick();
        Arm_Fb_RequestAll();
    }

    /* 同上：J1 用软件累计位置，不要求 0x36 回读 */
    if (Arm_Fb_GetJointsMask(q, &mask) == 0 || (mask & 0x0EU) != 0x0EU)
    {
        return;
    }

    if (Arm_Fb_GetStepPosition(&q[ARM_AXIS_J1]) != 0)
    {
        q[ARM_AXIS_J1] = s_target[ARM_AXIS_J1];
    }

    ok = 1U;

    if (fabsf(Math_Wrap180(s_target[ARM_AXIS_J1] - q[ARM_AXIS_J1]))
        > ARM_PTP_POS_TOL_DEG)
    {
        ok = 0U;
    }

    for (i = ARM_AXIS_J2; i < ARM_AXIS_NUM; i++)
    {
        if (fabsf(s_target[i] - q[i]) > ARM_PTP_POS_TOL_DEG)
        {
            ok = 0U;
        }
    }

    if (ok != 0U)
    {
        s_settle_count++;
        if (s_settle_count >= ARM_PTP_SETTLE_SAMPLES)
        {
            s_state = MOTION_DONE;
        }
    }
    else
    {
        s_settle_count = 0U;
    }
}

uint8_t Arm_Motion_IsBusy(void)
{
    return (s_state == MOTION_BUSY) ? 1U : 0U;
}

uint8_t Arm_Motion_IsDone(void)
{
    return (s_state == MOTION_DONE) ? 1U : 0U;
}

uint8_t Arm_Motion_HasFault(void)
{
    return (s_state == MOTION_FAULT) ? 1U : 0U;
}

Arm_Status_e Arm_Motion_GetError(void)
{
    return s_fault;
}

void Arm_Motion_Abort(void)
{
    (void)Arm_Step_StopNow();
    Arm_Servo_StopAll();

    s_state = MOTION_IDLE;
    s_fault = ARM_OK;
    s_settle_count = 0U;
}
