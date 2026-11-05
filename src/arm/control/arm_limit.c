/**
  ******************************************************************************
  * @file    arm/control/arm_limit.c
  ******************************************************************************
  */

#include "arm/control/arm_limit.h"
#include "arm/control/arm_kin.h"
#include "arm/arm_config.h"

#include <math.h>

/* 审计 H4：`q < MIN || q > MAX` 对 NaN 恒为假，NaN 会穿到 PWM 换算再被
   lroundf 变成垃圾值。改成「正向区间取反」，NaN 直接落到失败分支。 */
static int joint_in_range(float q, float q_min, float q_max)
{
    return (isfinite(q) && q >= q_min && q <= q_max) ? 1 : 0;
}

int Arm_Limit_CheckJoints(const float q_deg[4])
{
    if (q_deg == NULL)
    {
        return -1;
    }

    if (joint_in_range(q_deg[0],
                       ARM_Q1_MIN_DEG, ARM_Q1_MAX_DEG) == 0)
    {
        return -1;
    }
    if (joint_in_range(q_deg[1],
                       ARM_Q2_MIN_DEG, ARM_Q2_MAX_DEG) == 0)
    {
        return -1;
    }
    if (joint_in_range(q_deg[2],
                       ARM_Q3_MIN_DEG, ARM_Q3_MAX_DEG) == 0)
    {
        return -1;
    }
    if (joint_in_range(q_deg[3],
                       ARM_Q4_MIN_DEG, ARM_Q4_MAX_DEG) == 0)
    {
        return -1;
    }

    return 0;
}

int Arm_Limit_CheckPose(const Arm_Pose_t *pose)
{
    float q[4];
    float r;
    float z;

    if (pose == NULL)
    {
        return -1;
    }

    r = sqrtf(pose->x_mm * pose->x_mm + pose->y_mm * pose->y_mm);
    z = pose->z_mm;

    /* 同上（审计 H4）：NaN 不得穿过工作空间判断 */
    if (joint_in_range(r, ARM_R_MIN_MM, ARM_R_MAX_MM) == 0
        || joint_in_range(z, ARM_Z_MIN_MM, ARM_Z_MAX_MM) == 0)
    {
        return -1;
    }

    if (Arm_Kin_Inverse(pose, q) != 0)
    {
        return -1;
    }

    return Arm_Limit_CheckJoints(q);
}
