/**
  ******************************************************************************
  * @file    chassis/perception/odom.c
  ******************************************************************************
  */

#include "chassis/perception/odom.h"
#include "chassis/chassis_config.h"
#include "chassis/driver/encoder.h"
#include "chassis/driver/gyro.h"
#include "common/math_util.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

static Chassis_Odom_State_t s_odom;
static float    s_last_yaw_raw;
static float    s_seg_yaw_base;
static float    s_vl_filt;
static float    s_vr_filt;
static float    s_gyro_rate_filt;
static uint32_t s_prev_tick;
static uint8_t  s_first;
static volatile uint8_t s_mark_pending;

void Chassis_Odom_Init(void)
{
    float yaw;

    memset(&s_odom, 0, sizeof(s_odom));

    yaw = Chassis_Gyro_GetYawDeg();
    s_last_yaw_raw = yaw;
    s_seg_yaw_base = 0.0f;
    s_vl_filt = 0.0f;
    s_vr_filt = 0.0f;
    s_gyro_rate_filt = 0.0f;
    s_prev_tick = HAL_GetTick();
    s_first = 1U;
    s_mark_pending = 0U;
}

void Chassis_Odom_Update(void)
{
    Chassis_Odom_State_t next;
    uint32_t now;
    float dt;
    float dt_s;
    int16_t d_l;
    int16_t d_r;
    float mm_l;
    float mm_r;
    float dd;
    float dth;
    float v_l_raw;
    float v_r_raw;
    float alpha_v;
    float raw_yaw;
    float dyaw;
    float yaw_gain;
    float th_mid;

    now = HAL_GetTick();
    dt = (float)(now - s_prev_tick);
    if (dt < 1.0f)
    {
        dt = 1.0f;
    }
    s_prev_tick = now;
    dt_s = dt * 0.001f;

    d_l = Chassis_Encoder_ReadDelta(CHASSIS_ENC_LEFT);
    d_r = Chassis_Encoder_ReadDelta(CHASSIS_ENC_RIGHT);

    mm_l = (float)d_l * CHASSIS_MM_PER_PULSE;
    mm_r = (float)d_r * CHASSIS_MM_PER_PULSE;

    dd = (mm_l + mm_r) * 0.5f;
    dth = (mm_r - mm_l) / CHASSIS_TRACK_MM;   /* rad */

    v_l_raw = mm_l / dt_s;
    v_r_raw = mm_r / dt_s;

    /* 一阶低通，抑制 M 法量化噪声 */
    if (CHASSIS_SPEED_LPF_TAU_MS > 0.0f)
    {
        alpha_v = dt / (CHASSIS_SPEED_LPF_TAU_MS + dt);
        s_vl_filt += alpha_v * (v_l_raw - s_vl_filt);
        s_vr_filt += alpha_v * (v_r_raw - s_vr_filt);
    }
    else
    {
        s_vl_filt = v_l_raw;
        s_vr_filt = v_r_raw;
    }

    /* 取旧快照：临界区只做结构体拷贝，浮点运算全部在区外 */
    taskENTER_CRITICAL();
    next = s_odom;
    taskEXIT_CRITICAL();

    next.v_l_mmps = s_vl_filt;
    next.v_r_mmps = s_vr_filt;
    next.v_mmps = (s_vl_filt + s_vr_filt) * 0.5f;
    next.w_degps = dth / dt_s * MATH_RAD2DEG;

    /* 陀螺仪航向：逐帧取增量，无界累积，不丢圈 */
    raw_yaw = Chassis_Gyro_GetYawDeg();

    if (s_first != 0U)
    {
        s_last_yaw_raw = raw_yaw;
        s_first = 0U;
        dyaw = 0.0f;
    }
    else
    {
        dyaw = Math_Wrap180(raw_yaw - s_last_yaw_raw);
        s_last_yaw_raw = raw_yaw;
    }

    next.yaw_gyro_deg += dyaw;
    next.yaw_enc_deg += dth * MATH_RAD2DEG;

    /* wz 原始角速度积分：独立于模块姿态解算的对照航向，陀螺掉线时冻结 */
    if (Chassis_Gyro_IsOnline() != 0U)
    {
        next.yaw_wz_deg += Chassis_Gyro_GetWzDps() * dt_s;
    }

    /* 陀螺仪角度微分得到角速度：不依赖轮速，打滑时仍反映真实转向 */
    {
        float rate_raw = dyaw / dt_s;
        float a_rate;

        if (CHASSIS_GYRO_RATE_LPF_TAU_MS > 0.0f)
        {
            a_rate = dt / (CHASSIS_GYRO_RATE_LPF_TAU_MS + dt);
            s_gyro_rate_filt += a_rate * (rate_raw - s_gyro_rate_filt);
        }
        else
        {
            s_gyro_rate_filt = rate_raw;
        }

        next.yaw_rate_dps = s_gyro_rate_filt;
    }

    /* 陀螺仪掉线时完全依赖编码器航向，避免融合系数把航向冻住 */
    if (Chassis_Gyro_IsOnline() != 0U)
    {
        yaw_gain = CHASSIS_ODOM_YAW_FUSE_ALPHA;
    }
    else
    {
        yaw_gain = 0.0f;
    }
    next.yaw_deg += yaw_gain * dyaw
                  + (1.0f - yaw_gain) * (dth * MATH_RAD2DEG);

    /* 中点法二阶积分 */
    th_mid = (next.yaw_deg - dth * MATH_RAD2DEG * 0.5f) * MATH_DEG2RAD;
    next.x_mm += dd * cosf(th_mid);
    next.y_mm += dd * sinf(th_mid);

    next.dist_total_mm += dd;
    next.ds_seg_mm += dd;
    next.stamp_ms = now;
    next.sample_period_ms = (uint32_t)dt;

    /* 一次性写回；段起点打点在同一临界区内原子完成 */
    taskENTER_CRITICAL();
    if (s_mark_pending != 0U)
    {
        s_seg_yaw_base = next.yaw_deg;
        next.ds_seg_mm = 0.0f;
        s_mark_pending = 0U;
    }
    next.yaw_seg_deg = next.yaw_deg - s_seg_yaw_base;
    s_odom = next;
    taskEXIT_CRITICAL();
}

void Chassis_Odom_GetState(Chassis_Odom_State_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_odom;
    taskEXIT_CRITICAL();
}

void Chassis_Odom_Reset(float x_mm, float y_mm, float yaw_deg)
{
    taskENTER_CRITICAL();
    s_odom.x_mm = x_mm;
    s_odom.y_mm = y_mm;
    s_odom.yaw_deg = yaw_deg;
    s_odom.yaw_gyro_deg = yaw_deg;
    s_odom.yaw_enc_deg = yaw_deg;
    s_odom.yaw_wz_deg = yaw_deg;
    s_odom.dist_total_mm = 0.0f;
    s_odom.ds_seg_mm = 0.0f;
    s_odom.yaw_seg_deg = 0.0f;
    s_seg_yaw_base = yaw_deg;
    s_mark_pending = 0U;
    taskEXIT_CRITICAL();
}

void Chassis_Odom_MarkSegStart(void)
{
    /* 由下一次感知更新在临界区内原子清零，避免与增量累加竞争 */
    taskENTER_CRITICAL();
    s_mark_pending = 1U;
    taskEXIT_CRITICAL();
}

float Chassis_Odom_GetYawUnwrapped(void)
{
    float yaw;

    taskENTER_CRITICAL();
    yaw = s_odom.yaw_deg;
    taskEXIT_CRITICAL();

    return yaw;
}
