/**
  ******************************************************************************
  * @file    math_util.c
  ******************************************************************************
  */

#include "common/math_util.h"

float Math_Wrap180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg <= -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

float Math_Wrap360(float deg)
{
    deg = Math_Wrap180(deg);
    if (deg < 0.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

int Math_AngNear(float a, float b, float tol_deg)
{
    float d = Math_Wrap180(a - b);
    return (d <= tol_deg && d >= -tol_deg) ? 1 : 0;
}

float Math_Clamp(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

int16_t Math_ClampI16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < (int32_t)lo)
    {
        return lo;
    }
    if (v > (int32_t)hi)
    {
        return hi;
    }
    return (int16_t)v;
}
