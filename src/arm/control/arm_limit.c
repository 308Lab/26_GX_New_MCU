/**
  ******************************************************************************
  * @file    arm/control/arm_limit.c
  ******************************************************************************
  */

#include "arm/control/arm_limit.h"
#include "arm/control/arm_kin.h"
#include "arm/arm_config.h"

#include <math.h>

int Arm_Limit_CheckJoints(const float q_deg[4])
{
    if (q_deg == NULL)
    {
        return -1;
    }

    if (q_deg[0] < ARM_Q1_MIN_DEG || q_deg[0] > ARM_Q1_MAX_DEG)
    {
        return -1;
    }
    if (q_deg[1] < ARM_Q2_MIN_DEG || q_deg[1] > ARM_Q2_MAX_DEG)
    {
        return -1;
    }
    if (q_deg[2] < ARM_Q3_MIN_DEG || q_deg[2] > ARM_Q3_MAX_DEG)
    {
        return -1;
    }
    if (q_deg[3] < ARM_Q4_MIN_DEG || q_deg[3] > ARM_Q4_MAX_DEG)
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

    if (r < ARM_R_MIN_MM || r > ARM_R_MAX_MM
        || z < ARM_Z_MIN_MM || z > ARM_Z_MAX_MM)
    {
        return -1;
    }

    if (Arm_Kin_Inverse(pose, q) != 0)
    {
        return -1;
    }

    return Arm_Limit_CheckJoints(q);
}
