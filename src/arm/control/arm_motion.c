/**
  ******************************************************************************
  * @file    arm/control/arm_motion.c
  * @brief   PTP 多轴时间对齐：舵机同步帧 + 步进位置模式，同一拍下发。
  *
  * 2026-11-06 审计批A：
  *   H1  故障 = 停住并保持（发保持帧），不再 Arm_Servo_StopAll() 松手掉臂；
  *       只有 Arm_Stop()（人工急停）保留快速失能。
  *   H4  限位判断与 PWM 换算对 NaN 免疫（isfinite + 正向区间取反）。
  *   H10 指令发布顺序改为「先写数据 → 发送 → 最后置 BUSY」，
  *       并用 s_commanding 挡住第二条指令穿插发送半帧。
  *   H11 同步帧无长度无 ACK，下发后用 PRAD 回读位移确认帧真的被接受。
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

#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_BUSY,
    MOTION_DONE,
    MOTION_HOLD      /* 故障后已发保持帧：不下电、不再自动续动，等人工指令 */
} Motion_State_e;

static Motion_State_e s_state;
static Arm_Status_e   s_fault;

static float    s_target[4];
static uint32_t s_start_ms;
static uint32_t s_motion_time_ms;
static uint32_t s_timeout_ms;
static uint32_t s_last_fb_req_ms;
static uint8_t  s_settle_count;

/* 1 = 正在下发一条指令（数据已写、帧未发完），第二条 StartJoint 必须让路 */
static volatile uint8_t s_commanding;

/* H11：发帧前的回读基线，以及窗口内要看到的位移阈值 */
static uint16_t s_verify_ref_pwm[3];
static uint16_t s_verify_need_pwm[3];
static uint32_t s_verify_ref_stamp[3];
static uint8_t  s_verify_mask;
static uint8_t  s_verify_active;
static uint32_t s_verify_deadline_ms;

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

/* 从「目标关节角」反算 PWM；NaN / 越界一律判失败（审计 H4）。 */
static int motion_pwm_of(uint8_t axis, float q_deg, uint16_t *pwm)
{
    float f;

    if (pwm == NULL)
    {
        return -1;
    }

    f = servo_zero_pwm(axis) + q_deg * servo_pwm_per_deg(axis) * servo_dir(axis);

    /* 正向区间取反：NaN 落到失败分支，不能穿到 lroundf */
    if (isfinite(f) == 0
        || f < (float)ARM_SERVO_PWM_MIN
        || f > (float)ARM_SERVO_PWM_MAX)
    {
        return -1;
    }

    *pwm = (uint16_t)lroundf(f);

    /* lroundf 之后再兜一次：整数化溢出/四舍五入边界 */
    return (*pwm >= ARM_SERVO_PWM_MIN && *pwm <= ARM_SERVO_PWM_MAX) ? 0 : -1;
}

/* 故障保持：按最后一次回读的 PWM 发一帧「目标=当前」的保持帧（审计 H1）。
   回读缺失时退回本拍的目标角，绝不下发失能。 */
static void motion_hold_frame(void)
{
    uint16_t pwm[3];
    uint8_t  i;

    for (i = 0U; i < 3U; i++)
    {
        uint8_t  axis = (uint8_t)(ARM_AXIS_J2 + i);
        uint16_t cur = 0U;

        if (Arm_Fb_GetServoPwm(axis, &cur) != 0
            && motion_pwm_of(axis, s_target[axis], &cur) != 0)
        {
            /* 连目标都没有（例如初始化即故障）：取零位，仍是「保持」不是「松手」 */
            cur = (uint16_t)lroundf(servo_zero_pwm(axis));
        }

        pwm[i] = cur;
    }

    (void)Arm_Servo_SetPwmSync(s_servo_ids, pwm, 3U, ARM_PTP_HOLD_TIME_MS);
}

static void motion_fault(Arm_Status_e err)
{
    s_fault = err;
    s_verify_active = 0U;
    s_commanding = 0U;
    (void)Arm_Step_StopNow();
    motion_hold_frame();          /* 先停住并保持，再进 HOLD 状态（审计 H1） */
    s_state = MOTION_HOLD;
}

/* H11：记录发帧前的回读基线与需要校验的轴。
   只有「目标与当前差 ≥ ARM_PTP_VERIFY_MIN_MOVE_PWM」的轴参与校验：
   微动（<2.7°）在 900 ms 窗口内本来就看不出来。 */
static void motion_verify_prepare(const uint16_t pwm[3])
{
    uint8_t i;

    s_verify_mask = 0U;
    s_verify_active = 0U;

    for (i = 0U; i < 3U; i++)
    {
        uint8_t  axis = (uint8_t)(ARM_AXIS_J2 + i);
        uint16_t cur = 0U;
        int32_t  diff;

        s_verify_ref_stamp[i] = Arm_Servo_GetRxStampMs(s_servo_ids[i]);
        s_verify_ref_pwm[i] = 0U;
        s_verify_need_pwm[i] = ARM_PTP_VERIFY_MIN_PWM;

        if (Arm_Fb_GetServoPwm(axis, &cur) != 0)
        {
            continue;
        }

        s_verify_ref_pwm[i] = cur;

        diff = (int32_t)pwm[i] - (int32_t)cur;
        if (diff < 0)
        {
            diff = -diff;
        }

        if (diff < (int32_t)ARM_PTP_VERIFY_MIN_MOVE_PWM)
        {
            continue;
        }

        /* 窗口约为运动时间的 1/4，所以要求看到约 1/4 的位移 */
        s_verify_need_pwm[i] = (uint16_t)(diff / 4);
        s_verify_mask |= (uint8_t)(1U << i);
    }

    if (s_verify_mask != 0U)
    {
        s_verify_active = 1U;
        s_verify_deadline_ms = HAL_GetTick() + ARM_PTP_VERIFY_WINDOW_MS;
    }
}

/* 至少一轴「有新回读 + 位移达到阈值」才认为同步帧被接受 */
static uint8_t motion_verify_ok(void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        uint8_t  axis = (uint8_t)(ARM_AXIS_J2 + i);
        uint16_t cur = 0U;
        int32_t  diff;

        if ((s_verify_mask & (uint8_t)(1U << i)) == 0U)
        {
            continue;
        }

        if (Arm_Servo_GetRxStampMs(s_servo_ids[i]) == s_verify_ref_stamp[i])
        {
            continue;      /* 还是旧缓存，不能当证据 */
        }

        if (Arm_Fb_GetServoPwm(axis, &cur) != 0)
        {
            continue;
        }

        diff = (int32_t)cur - (int32_t)s_verify_ref_pwm[i];
        if (diff < 0)
        {
            diff = -diff;
        }

        if (diff >= (int32_t)s_verify_need_pwm[i])
        {
            return 1U;
        }
    }

    return 0U;
}

/* 放弃当前这条指令的下发权（不改状态机） */
static void motion_release_command(void)
{
    taskENTER_CRITICAL();
    s_commanding = 0U;
    taskEXIT_CRITICAL();
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
    s_commanding = 0U;
    s_verify_mask = 0U;
    s_verify_active = 0U;
    s_verify_deadline_ms = 0U;

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

    /* 一条指令在途：BUSY 或上一帧还没发完都不接新指令（审计 H10） */
    taskENTER_CRITICAL();
    if (s_state == MOTION_BUSY || s_commanding != 0U)
    {
        taskEXIT_CRITICAL();
        return ARM_BUSY;
    }
    s_commanding = 1U;
    taskEXIT_CRITICAL();

    if (Arm_Limit_CheckJoints(q_target) != 0)
    {
        motion_release_command();
        return ARM_ERR_LIMIT;
    }

    /* J1 无绝对零位，且部分驱动固件不响应 0x36 位置回读：
       J1 用软件累计位置，只要求 J2/J3/J4 三轴回读就绪。 */
    if (Arm_Fb_GetJointsMask(q_now, &mask) == 0 || (mask & 0x0EU) != 0x0EU)
    {
        /* 反馈未就绪：发起一轮回读，调用方下一拍重试 */
        motion_release_command();
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
        motion_release_command();
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

    if (isfinite(t_servo) == 0 || t_servo > 9.0f)
    {
        motion_release_command();
        return ARM_ERR_PARAM;
    }

    servo_time_ms = (uint16_t)lroundf(t_servo * 1000.0f);

    for (i = 0U; i < 3U; i++)
    {
        uint8_t axis = (uint8_t)(ARM_AXIS_J2 + i);

        if (motion_pwm_of(axis, q_target[axis], &pwm[i]) != 0)
        {
            motion_release_command();
            return ARM_ERR_LIMIT;
        }
    }

    /* 审计 H10：先把这条指令的全部数据写好，最后才置 BUSY。
       否则 BUSY 先发布、发送又阻塞 ~7 ms，Arm_Motion_Update 会拿
       0 的 s_start_ms 算出超大 elapsed，立刻判 TIMEOUT 并掉臂。 */
    s_start_ms = HAL_GetTick();
    s_motion_time_ms = (uint32_t)lroundf(t_servo * 1000.0f);
    s_timeout_ms = s_motion_time_ms + ARM_PTP_TIMEOUT_MARGIN_MS;
    s_last_fb_req_ms = s_start_ms;
    s_settle_count = 0U;
    s_fault = ARM_OK;

    for (i = 0U; i < ARM_AXIS_NUM; i++)
    {
        s_target[i] = q_target[i];
    }

    /* H11：发帧前记下回读基线 */
    motion_verify_prepare(pwm);

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

    /* 数据与帧都已落地，原子发布 BUSY 并交出下发权 */
    taskENTER_CRITICAL();
    s_state = MOTION_BUSY;
    s_commanding = 0U;
    taskEXIT_CRITICAL();

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

    /* H11：同步帧无 ACK，先确认帧被舵机接受（保持扭矩，不是 H1 的失能） */
    if (s_verify_active != 0U)
    {
        if (motion_verify_ok() != 0U)
        {
            s_verify_active = 0U;
        }
        else if ((int32_t)(HAL_GetTick() - s_verify_deadline_ms) >= 0)
        {
            motion_fault(ARM_ERR_COMM);
            return;
        }
        else
        {
            /* 窗口内继续等回读 */
        }
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
    return (s_state == MOTION_HOLD) ? 1U : 0U;
}

Arm_Status_e Arm_Motion_GetError(void)
{
    return s_fault;
}

void Arm_Motion_Abort(void)
{
    (void)Arm_Step_StopNow();

    /* 审计 H1：中止 = 停在原地保持，不再 StopAll() 松手。
       需要断扭矩的人工急停走 Arm_Stop()。 */
    motion_hold_frame();

    s_state = MOTION_IDLE;
    s_fault = ARM_OK;
    s_settle_count = 0U;
    s_verify_active = 0U;
    motion_release_command();
}
