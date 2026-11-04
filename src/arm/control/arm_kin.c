/**
  ******************************************************************************
  * @file    arm/control/arm_kin.c
  * @brief   底座旋转 + 带关节偏置的 2R 俯仰臂 + 腕部 TCP 的正解/逆解。
  *
  * 俯仰面坐标 (r, z)：
  *   θ2 轴在 (0, H0)；大臂沿杆 A=120，末端垂直偏置 B=20 到 θ3 轴；
  *   θ4 轴 = θ2 + A·u(q2) + B·n(q2) + C·u(q2+q3)；
  *   TCP = θ4 + R(q2+q3+q4)·(F, U)。
  *   假设 q3=0 时小臂与大臂同向（直臂）。
  ******************************************************************************
  */

#include "arm/control/arm_kin.h"
#include "arm/arm_config.h"
#include "common/math_util.h"

#include <math.h>

#define KIN_EPS (1.0e-6f)

int Arm_Kin_Forward(const float q_deg[4], Arm_Pose_t *pose)
{
    float q1;
    float q2;
    float q3;
    float q4;
    float phi;
    float r;
    float z;

    if (q_deg == NULL || pose == NULL)
    {
        return -1;
    }

    q1 = q_deg[0] * MATH_DEG2RAD;
    q2 = q_deg[1] * MATH_DEG2RAD;
    q3 = q_deg[2] * MATH_DEG2RAD;
    q4 = q_deg[3] * MATH_DEG2RAD;
    /* 小臂方向角定义为 q2 − q3：实机 q3 增大 = 小臂相对大臂向前折 */
    phi = q2 - q3 + q4;

    r = ARM_UPPER_ALONG_MM * cosf(q2)
      - ARM_ELBOW_OFFSET_MM * ARM_ELBOW_OFFSET_SIGN * sinf(q2)
      + ARM_FOREARM_MM * cosf(q2 - q3)
      + ARM_TCP_FWD_MM * cosf(phi)
      - ARM_TCP_UP_MM * sinf(phi);

    z = ARM_BASE_H_MM
      + ARM_UPPER_ALONG_MM * sinf(q2)
      + ARM_ELBOW_OFFSET_MM * ARM_ELBOW_OFFSET_SIGN * cosf(q2)
      + ARM_FOREARM_MM * sinf(q2 - q3)
      + ARM_TCP_FWD_MM * sinf(phi)
      + ARM_TCP_UP_MM * cosf(phi);

    pose->x_mm = r * cosf(q1);
    pose->y_mm = r * sinf(q1);
    pose->z_mm = z;
    pose->tool_deg = q_deg[1] - q_deg[2] + q_deg[3];

    return 0;
}

int Arm_Kin_Inverse(const Arm_Pose_t *pose, float q_deg[4])
{
    float q1;
    float r;
    float phi;
    float r4;
    float z4;
    float a;
    float b;
    float c;
    float k;
    float m;
    float n;
    float rm;
    float ratio;
    float alpha;
    float q2_a;
    float q2_b;
    float q3_a;
    float q3_b;
    float q2;
    float q3;
    float q4;
    uint8_t pick_a;

    if (pose == NULL || q_deg == NULL)
    {
        return -1;
    }

    q1 = atan2f(pose->y_mm, pose->x_mm);
    r = sqrtf(pose->x_mm * pose->x_mm + pose->y_mm * pose->y_mm);
    phi = pose->tool_deg * MATH_DEG2RAD;

    /* 先扣掉腕部 TCP 偏移，得到 θ4 轴目标 */
    r4 = r - (ARM_TCP_FWD_MM * cosf(phi) - ARM_TCP_UP_MM * sinf(phi));
    z4 = pose->z_mm
       - (ARM_TCP_FWD_MM * sinf(phi) + ARM_TCP_UP_MM * cosf(phi))
       - ARM_BASE_H_MM;

    a = ARM_UPPER_ALONG_MM;
    b = ARM_ELBOW_OFFSET_MM * ARM_ELBOW_OFFSET_SIGN;
    c = ARM_FOREARM_MM;

    /* 解 (p·u - A)² + (p·n - B)² = C² */
    k = (r4 * r4 + z4 * z4 + a * a + b * b - c * c) * 0.5f;
    m = a * r4 + b * z4;
    n = a * z4 - b * r4;
    rm = sqrtf(m * m + n * n);

    if (rm < KIN_EPS)
    {
        return -1;
    }

    ratio = k / rm;
    if (!(ratio >= -1.0f && ratio <= 1.0f))
    {
        return -1;
    }

    alpha = atan2f(n, m);
    q2_a = alpha + acosf(ratio);
    q2_b = alpha - acosf(ratio);

    q3_a = atan2f(z4 * cosf(q2_a) - r4 * sinf(q2_a) - b,
                  r4 * cosf(q2_a) + z4 * sinf(q2_a) - a);
    q3_b = atan2f(z4 * cosf(q2_b) - r4 * sinf(q2_b) - b,
                  r4 * cosf(q2_b) + z4 * sinf(q2_b) - a);

    /* 按 ARM_IK_ELBOW 选择肘上/肘下 */
    pick_a = (((q3_a >= 0.0f) ? 1U : 0U)
              == ((ARM_IK_ELBOW >= 0.0f) ? 1U : 0U)) ? 1U : 0U;

    q2 = pick_a ? q2_a : q2_b;
    /* 内部解基于“小臂角 = q2 + q3”，输出关节角需取反 */
    q3 = -(pick_a ? q3_a : q3_b);
    q4 = phi - q2 + q3;

    q_deg[0] = q1 * MATH_RAD2DEG;
    q_deg[1] = q2 * MATH_RAD2DEG;
    q_deg[2] = q3 * MATH_RAD2DEG;
    q_deg[3] = q4 * MATH_RAD2DEG;

    return 0;
}
