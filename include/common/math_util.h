/**
  ******************************************************************************
  * @file    math_util.h
  * @brief   跨子系统通用数学工具：角度环绕、限幅。
  ******************************************************************************
  */

#ifndef COMMON_MATH_UTIL_H
#define COMMON_MATH_UTIL_H

#include <stdint.h>

#define MATH_PI        (3.14159265f)
#define MATH_2PI       (6.28318531f)
#define MATH_RAD2DEG   (57.29577951f)
#define MATH_DEG2RAD   (0.0174532925f)

/* 角度归一化到 (-180, 180] */
float Math_Wrap180(float deg);

/* 角度归一化到 [0, 360) */
float Math_Wrap360(float deg);

/* 两角之差经 Wrap180 后是否在容差内 */
int Math_AngNear(float a, float b, float tol_deg);

float Math_Clamp(float v, float lo, float hi);

int16_t Math_ClampI16(int32_t v, int16_t lo, int16_t hi);

#endif /* COMMON_MATH_UTIL_H */
