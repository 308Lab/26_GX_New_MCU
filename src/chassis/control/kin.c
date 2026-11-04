/**
  ******************************************************************************
  * @file    chassis/control/kin.c
  ******************************************************************************
  */

#include "chassis/control/kin.h"
#include "chassis/chassis_config.h"
#include "common/math_util.h"

#include <stddef.h>

void Chassis_Kin_Forward(float vL_mmps, float vR_mmps,
                         float *v_mmps, float *w_degps)
{
    if (v_mmps != NULL)
    {
        *v_mmps = (vL_mmps + vR_mmps) * 0.5f;
    }
    if (w_degps != NULL)
    {
        *w_degps = (vR_mmps - vL_mmps) / CHASSIS_TRACK_MM * MATH_RAD2DEG;
    }
}

void Chassis_Kin_Inverse(float v_mmps, float w_degps,
                         float *vL_mmps, float *vR_mmps)
{
    float half_w = w_degps * MATH_DEG2RAD * CHASSIS_TRACK_MM * 0.5f;

    if (vL_mmps != NULL)
    {
        *vL_mmps = v_mmps - half_w;
    }
    if (vR_mmps != NULL)
    {
        *vR_mmps = v_mmps + half_w;
    }
}

void Chassis_Kin_Spin(float w_degps, float *vL_mmps, float *vR_mmps)
{
    float half_w = w_degps * MATH_DEG2RAD * CHASSIS_TRACK_MM * 0.5f;

    if (vL_mmps != NULL)
    {
        *vL_mmps = -half_w;
    }
    if (vR_mmps != NULL)
    {
        *vR_mmps = half_w;
    }
}

void Chassis_Kin_Arc(float v_mmps, float radius_mm,
                     float *vL_mmps, float *vR_mmps)
{
    float w_degps;

    if (radius_mm > -1.0f && radius_mm < 1.0f)
    {
        /* 半径趋近 0：退化为原地旋转，方向由半径符号决定 */
        Chassis_Kin_Spin(v_mmps >= 0.0f ? CHASSIS_W_MAX_DPS : -CHASSIS_W_MAX_DPS,
                         vL_mmps, vR_mmps);
        return;
    }

    w_degps = v_mmps / radius_mm * MATH_RAD2DEG;
    Chassis_Kin_Inverse(v_mmps, w_degps, vL_mmps, vR_mmps);
}

float Chassis_Kin_RadiusFrom(float arc_len_mm, float angle_deg)
{
    if (angle_deg > -0.001f && angle_deg < 0.001f)
    {
        return 0.0f;
    }
    return arc_len_mm / (angle_deg * MATH_DEG2RAD);
}

float Chassis_Kin_MmToPulse(float mm)
{
    return mm * CHASSIS_PULSE_PER_MM;
}

float Chassis_Kin_PulseToMm(int32_t pulse)
{
    return (float)pulse * CHASSIS_MM_PER_PULSE;
}
