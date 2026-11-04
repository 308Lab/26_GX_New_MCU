/**
  ******************************************************************************
  * @file    arm/planning/arm_action.c
  * @brief   Home / Stow / Move / Pick&Place / Aim 的非阻塞状态机。
  ******************************************************************************
  */

#include "arm/planning/arm_action.h"
#include "arm/arm_config.h"
#include "arm/control/arm_kin.h"
#include "arm/control/arm_limit.h"
#include "arm/control/arm_motion.h"
#include "arm/control/arm_actuator.h"
#include "main.h"

typedef enum
{
    ACT_IDLE = 0,
    ACT_BUSY,
    ACT_DONE,
    ACT_FAULT
} Act_State_e;

static Act_State_e  s_state;
static Arm_Action_e s_action;
static Arm_Status_e s_error;

static uint32_t s_start_ms;
static uint32_t s_timeout_ms;
static uint8_t  s_step;

static float s_q_pick_above[4];
static float s_q_pick[4];
static float s_q_place_above[4];
static float s_q_place[4];

static void act_fault(Arm_Status_e err)
{
    Arm_Motion_Abort();
    (void)Arm_Actuator_Gripper(ARM_GRIPPER_HOLD);
    (void)Arm_Actuator_Laser(0U);

    s_error = err;
    s_state = ACT_FAULT;
}

static Arm_Status_e act_begin(Arm_Action_e action,
                              const float q_deg[4],
                              uint16_t timeout_ms)
{
    Arm_Status_e rc;

    if (s_state == ACT_BUSY)
    {
        return ARM_BUSY;
    }

    if (Arm_Limit_CheckJoints(q_deg) != 0)
    {
        return ARM_ERR_LIMIT;
    }

    rc = Arm_Motion_StartJoint(q_deg, ARM_PTP_MIN_TIME_S);
    if (rc != ARM_OK)
    {
        return rc;
    }

    s_action = action;
    s_state = ACT_BUSY;
    s_error = ARM_OK;
    s_step = 0U;
    s_start_ms = HAL_GetTick();
    s_timeout_ms = (timeout_ms != 0U)
                 ? (uint32_t)timeout_ms : ARM_DEBUG_ACTION_TIMEOUT_MS;

    return ARM_OK;
}

static int act_motion_done(void)
{
    if (Arm_Motion_HasFault() != 0U)
    {
        act_fault(Arm_Motion_GetError());
        return -1;
    }

    return (Arm_Motion_IsDone() != 0U) ? 1 : 0;
}

static int act_start_motion(const float q_deg[4])
{
    Arm_Status_e rc = Arm_Motion_StartJoint(q_deg, ARM_PTP_MIN_TIME_S);

    if (rc == ARM_BUSY)
    {
        return 0;
    }
    if (rc != ARM_OK)
    {
        act_fault(rc);
        return -1;
    }

    return 1;
}

static void pickplace_update(void)
{
    int done;

    switch (s_step)
    {
    case 0U:
        (void)Arm_Actuator_Gripper(ARM_GRIPPER_OPEN);
        s_step = 1U;
        break;

    case 1U:
        if (act_start_motion(s_q_pick_above) != 0)
        {
            s_step = 2U;
        }
        break;

    case 2U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 3U;
        }
        break;

    case 3U:
        if (act_start_motion(s_q_pick) != 0)
        {
            s_step = 4U;
        }
        break;

    case 4U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 5U;
        }
        break;

    case 5U:
        (void)Arm_Actuator_Gripper(ARM_GRIPPER_GRASP);
        s_step = 6U;
        break;

    case 6U:
        if (act_start_motion(s_q_pick_above) != 0)
        {
            s_step = 7U;
        }
        break;

    case 7U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 8U;
        }
        break;

    case 8U:
        if (act_start_motion(s_q_place_above) != 0)
        {
            s_step = 9U;
        }
        break;

    case 9U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 10U;
        }
        break;

    case 10U:
        if (act_start_motion(s_q_place) != 0)
        {
            s_step = 11U;
        }
        break;

    case 11U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 12U;
        }
        break;

    case 12U:
        (void)Arm_Actuator_Gripper(ARM_GRIPPER_OPEN);
        s_step = 13U;
        break;

    case 13U:
        if (act_start_motion(s_q_place_above) != 0)
        {
            s_step = 14U;
        }
        break;

    case 14U:
        done = act_motion_done();
        if (done > 0)
        {
            s_step = 15U;
        }
        break;

    default:
        s_state = ACT_DONE;
        break;
    }
}

void Arm_Act_Init(void)
{
    s_state = ACT_IDLE;
    s_action = ARM_ACT_NONE;
    s_error = ARM_OK;
    s_step = 0U;
    s_start_ms = 0U;
    s_timeout_ms = 0U;
}

Arm_Status_e Arm_Act_StartHome(uint16_t timeout_ms)
{
    /* J3 软限位为 35~179°，q3=0 不可达且对应 PWM 超出 500~2500；
       Home 不再使用全零，暂以收拢安全位为目标。 */
    float q[4] = {
        ARM_STOW_Q1_DEG,
        ARM_STOW_Q2_DEG,
        ARM_STOW_Q3_DEG,
        ARM_STOW_Q4_DEG
    };

    return act_begin(ARM_ACT_HOME, q, timeout_ms);
}

Arm_Status_e Arm_Act_StartStow(uint16_t timeout_ms)
{
    float q[4] = {
        ARM_STOW_Q1_DEG,
        ARM_STOW_Q2_DEG,
        ARM_STOW_Q3_DEG,
        ARM_STOW_Q4_DEG
    };

    return act_begin(ARM_ACT_STOW, q, timeout_ms);
}

Arm_Status_e Arm_Act_StartJoint(const float q_deg[4], uint16_t timeout_ms)
{
    return act_begin(ARM_ACT_JOINT, q_deg, timeout_ms);
}

Arm_Status_e Arm_Act_StartPose(const Arm_Pose_t *pose, uint16_t timeout_ms)
{
    float q[4];

    if (pose == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (Arm_Kin_Inverse(pose, q) != 0)
    {
        return ARM_ERR_UNREACHABLE;
    }

    if (Arm_Limit_CheckJoints(q) != 0)
    {
        return ARM_ERR_LIMIT;
    }

    return act_begin(ARM_ACT_POSE, q, timeout_ms);
}

Arm_Status_e Arm_Act_StartPickPlace(const Arm_Pose_t *pick,
                                    const Arm_Pose_t *place,
                                    float approach_mm,
                                    uint16_t timeout_ms)
{
    Arm_Pose_t pick_above;
    Arm_Pose_t place_above;

    if (pick == NULL || place == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (s_state == ACT_BUSY)
    {
        return ARM_BUSY;
    }

    pick_above = *pick;
    place_above = *place;
    pick_above.z_mm += approach_mm;
    place_above.z_mm += approach_mm;

    if (Arm_Limit_CheckPose(&pick_above) != 0
        || Arm_Limit_CheckPose(pick) != 0
        || Arm_Limit_CheckPose(&place_above) != 0
        || Arm_Limit_CheckPose(place) != 0)
    {
        return ARM_ERR_LIMIT;
    }

    if (Arm_Kin_Inverse(&pick_above, s_q_pick_above) != 0
        || Arm_Kin_Inverse(pick, s_q_pick) != 0
        || Arm_Kin_Inverse(&place_above, s_q_place_above) != 0
        || Arm_Kin_Inverse(place, s_q_place) != 0)
    {
        return ARM_ERR_UNREACHABLE;
    }

    s_action = ARM_ACT_PICK_PLACE;
    s_state = ACT_BUSY;
    s_error = ARM_OK;
    s_step = 0U;
    s_start_ms = HAL_GetTick();
    s_timeout_ms = (timeout_ms != 0U)
                 ? (uint32_t)timeout_ms : ARM_DEBUG_ACTION_TIMEOUT_MS;

    return ARM_OK;
}

Arm_Status_e Arm_Act_StartAim(const Arm_Pose_t *aim_pose, uint16_t timeout_ms)
{
    float q[4];

    if (aim_pose == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (Arm_Kin_Inverse(aim_pose, q) != 0)
    {
        return ARM_ERR_UNREACHABLE;
    }

    if (Arm_Limit_CheckJoints(q) != 0)
    {
        return ARM_ERR_LIMIT;
    }

    return act_begin(ARM_ACT_AIM, q, timeout_ms);
}

Arm_Status_e Arm_Act_Hold(void)
{
    if (s_state == ACT_BUSY)
    {
        return ARM_BUSY;
    }

    (void)Arm_Actuator_Gripper(ARM_GRIPPER_HOLD);
    s_action = ARM_ACT_HOLD;
    s_state = ACT_DONE;
    return ARM_OK;
}

Arm_Status_e Arm_Act_Release(void)
{
    if (s_state == ACT_BUSY)
    {
        return ARM_BUSY;
    }

    (void)Arm_Actuator_Gripper(ARM_GRIPPER_OPEN);
    s_action = ARM_ACT_RELEASE;
    s_state = ACT_DONE;
    return ARM_OK;
}

void Arm_Act_Update(void)
{
    if (s_state != ACT_BUSY)
    {
        return;
    }

    if ((HAL_GetTick() - s_start_ms) > s_timeout_ms)
    {
        act_fault(ARM_ERR_TIMEOUT);
        return;
    }

    switch (s_action)
    {
    case ARM_ACT_PICK_PLACE:
        pickplace_update();
        break;

    case ARM_ACT_AIM:
        if (act_motion_done() > 0)
        {
            (void)Arm_Actuator_Laser(1U);
            s_state = ACT_DONE;
        }
        break;

    case ARM_ACT_HOME:
    case ARM_ACT_STOW:
    case ARM_ACT_JOINT:
    case ARM_ACT_POSE:
        if (act_motion_done() > 0)
        {
            s_state = ACT_DONE;
        }
        break;

    default:
        s_state = ACT_DONE;
        break;
    }
}

uint8_t Arm_Act_IsBusy(void)
{
    return (s_state == ACT_BUSY) ? 1U : 0U;
}

Arm_Action_e Arm_Act_Current(void)
{
    return s_action;
}

Arm_Status_e Arm_Act_GetError(void)
{
    return s_error;
}

void Arm_Act_Abort(void)
{
    Arm_Motion_Abort();
    (void)Arm_Actuator_Gripper(ARM_GRIPPER_HOLD);
    (void)Arm_Actuator_Laser(0U);

    s_state = ACT_IDLE;
    s_action = ARM_ACT_NONE;
    s_error = ARM_OK;
    s_step = 0U;
}
