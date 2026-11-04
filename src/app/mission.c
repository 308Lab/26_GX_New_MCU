/**
  ******************************************************************************
  * @file    app/mission.c
  * @brief   正式模式任务层：A/B/C/D 编排、视觉事件消费、失败降级。
  *
  * 与 Vision_Link_Update() 同任务上下文运行（MainTask / MissionTask）。
  * 路径事件与视觉事件的唯一消费者。
  ******************************************************************************
  */

#include "app/mission.h"
#include "app/mission_config.h"
#include "app/vision_link.h"
#include "arm/arm.h"
#include "chassis/chassis.h"
#include "chassis/chassis_config.h"
#include "main.h"

#include <math.h>
#include <stdlib.h>

static Mission_State_e s_state;
static uint8_t  s_sub;
static uint32_t s_t_ms;
static uint32_t s_sub_ms;
static uint32_t s_state_ms;

static uint8_t  s_rank;
static uint8_t  s_arrive_id;
static uint8_t  s_scan_result;   /* 0=等待 1=成功 2=错误 3=超时 */
static uint8_t  s_vision_err;
static uint8_t  s_fail;
static uint8_t  s_conv;
static uint8_t  s_track_retry;
static uint8_t  s_track_lost;
static uint8_t  s_target_down;
static uint8_t  s_path_done;
static uint8_t  s_link_down;
static uint32_t s_link_ms;
static uint8_t  s_abort_req;
static uint8_t  s_degraded;
static uint8_t  s_servo_busy;
static uint8_t  s_servo_pending;

static const uint16_t POSE_COMMON[3] = MISSION_POSE_COMMON_OPEN;
static const uint16_t POSE_B[3]      = MISSION_POSE_B_PRESET;
static const uint16_t POSE_B_BUCKET[3] = MISSION_POSE_B_BUCKET;
static const uint16_t POSE_C[3]      = MISSION_POSE_C_PRESET;
static const uint16_t POSE_D[3]      = MISSION_POSE_D_PRESET;
static const uint16_t POSE_STOW[3]   = MISSION_POSE_STOW;

static const float A_Q[4] = {
    0.0f,
    MISSION_A_ARM_Q2_DEG,
    MISSION_A_ARM_Q3_DEG,
    MISSION_A_ARM_Q4_DEG
};

/* ===================== 辅助 ===================== */

static Vision_Zone_e zone_of(uint8_t pid)
{
    switch (pid)
    {
    case (uint8_t)'B': return VISION_ZONE_B;
    case (uint8_t)'C': return VISION_ZONE_C;
    case (uint8_t)'D': return VISION_ZONE_D;
    default:           return VISION_ZONE_NONE;
    }
}

static void mission_enter(Mission_State_e st)
{
    s_state = st;
    s_sub = 0U;
    s_sub_ms = HAL_GetTick();
    s_state_ms = HAL_GetTick();
    s_t_ms = HAL_GetTick();
    s_conv = 0U;
    s_track_retry = 0U;
}

static void mission_hard_stop(void)
{
    Chassis_Path_Abort();
    Arm_Stop();
    (void)Arm_Laser(0U);
}

/* ===================== 事件泵 ===================== */

static void mission_pump(void)
{
    Path_Event_t pe;
    Vision_Event_t ve;

    while (Chassis_Path_TakeEvent(&pe))
    {
        switch (pe.type)
        {
        case PATH_EV_SCAN_BEGIN:
            if (s_degraded == 0U)
            {
                (void)Vision_Link_StartScan(zone_of(pe.point_id));
                (void)Vision_Link_RequestOrder(zone_of(pe.point_id));
            }
            break;

        case PATH_EV_ARRIVE_POINT:
            s_arrive_id = pe.point_id;
            break;

        case PATH_EV_AUX_DONE:
            s_servo_busy = 0U;
            s_servo_pending = 0U;
            break;

        case PATH_EV_DONE:
            s_path_done = 1U;
            break;

        case PATH_EV_ERROR:
            mission_hard_stop();
            mission_enter(MISSION_STATE_ERROR);
            break;

        default:
            break;
        }
    }

    while (Vision_Link_TakeEvent(&ve))
    {
        switch (ve.type)
        {
        case VISION_EV_SCAN_OK:
            s_scan_result = 1U;
            break;

        case VISION_EV_SCAN_ERROR:
            s_vision_err = ve.value;
            s_scan_result = 2U;
            break;

        case VISION_EV_SCAN_TIMEOUT:
            s_scan_result = 3U;
            break;

        case VISION_EV_ORDER:
            s_rank = ve.value;
            if (s_rank > 2U)
            {
                s_rank = 1U;
            }
            /* 在扫描倒回之前设置 rank，决定一次性倒回距离 */
            Chassis_Path_SetRank(s_rank);
            break;

        case VISION_EV_TARGET_LOST:
            break;

        case VISION_EV_TRACK_LOST_5X:
            s_track_lost = 1U;
            break;

        case VISION_EV_C_TARGET_DOWN:
            s_target_down = 1U;
            break;

        case VISION_EV_LINK_LOST:
            s_link_down = 1U;
            s_link_ms = HAL_GetTick();
            break;

        case VISION_EV_LINK_RECOVERED:
            s_link_down = 0U;
            break;

        default:
            break;
        }
    }
}

/* ===================== 追踪闭环 ===================== */
/* 返回 0=进行中，1=收敛，-1=失败 */

static int mission_track_step(Vision_TrackTarget_e target, uint16_t timeout_ms)
{
    Vision_TrackSample_t ts;

    if (s_track_lost != 0U)
    {
        s_track_lost = 0U;
        (void)Vision_Link_TrackStop(target);
        return -1;
    }

    if ((HAL_GetTick() - s_sub_ms) > timeout_ms)
    {
        (void)Vision_Link_TrackStop(target);
        return -1;
    }

    if (Arm_IsBusy() != 0U)
    {
        return 0;
    }

    if (Vision_Link_GetTrack(target, &ts) == 0U || ts.valid == 0U)
    {
        s_conv = 0U;
        return 0;
    }

    if (abs((int)ts.dx_mm) <= MISSION_TRACK_TOL_MM
        && abs((int)ts.dy_mm) <= MISSION_TRACK_TOL_MM)
    {
        s_conv++;
        if (s_conv >= MISSION_TRACK_CONV_SAMPLES)
        {
            (void)Vision_Link_TrackStop(target);
            s_conv = 0U;
            return 1;
        }
    }
    else
    {
        s_conv = 0U;
        (void)Arm_NudgeTool(ts.dx_mm, ts.dy_mm, 2000U);
    }

    return 0;
}

/* ===================== 底盘到桶 ===================== */

static int16_t mission_bucket_offset_mm(void)
{
    if (s_rank == 0U)
    {
        return 150;     /* 近点多退了 150，需前进 */
    }
    if (s_rank == 2U)
    {
        return -150;    /* 远点少退了 150，需后退 */
    }
    return 0;
}

/* ===================== 状态机 ===================== */

static void mission_run_a(uint32_t now)
{
    switch (s_sub)
    {
    case 0U:
        if (Arm_IsBusy() != 0U)
        {
            break;   /* 等摆臂完成 */
        }
        (void)Arm_MoveBaseRelative(-90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 1U;
        break;

    case 1U:
        if ((int32_t)(now - s_t_ms) < 0)
        {
            break;
        }
        s_scan_result = 0U;
        s_fail = 0U;
        if (Vision_Link_RequestScanCode() != VISION_LINK_OK)
        {
            s_scan_result = 3U;   /* 发送失败按超时计一次 */
        }
        s_t_ms = now + MISSION_SCAN_CODE_TIMEOUT_MS;
        s_sub = 2U;
        break;

    case 2U:
        if (s_scan_result == 0U)
        {
            if ((int32_t)(now - s_t_ms) < 0)
            {
                break;
            }
            s_scan_result = 3U;
        }

        if (s_scan_result == 1U)
        {
            (void)Arm_MoveBaseRelative(90.0f);
            s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
            s_sub = 3U;
        }
        else if (s_scan_result == 2U
                 && s_vision_err >= 1U && s_vision_err <= 3U)
        {
            /* 配置类错误：立即降级 */
            s_degraded = 1U;
            mission_enter(MISSION_STATE_DEGRADED);
        }
        else
        {
            s_fail++;
            if (s_fail >= 3U)
            {
                s_degraded = 1U;
                mission_enter(MISSION_STATE_DEGRADED);
            }
            else
            {
                s_scan_result = 0U;
                (void)Vision_Link_RequestScanCode();
                s_t_ms = now + MISSION_SCAN_CODE_TIMEOUT_MS;
                s_sub = 2U;
            }
        }
        break;

    case 3U:
        if ((int32_t)(now - s_t_ms) < 0)
        {
            break;
        }
        (void)Arm_Close();
        (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 4U;
        break;

    case 4U:
        if ((int32_t)(now - s_t_ms) < 0)
        {
            break;
        }
        (void)Chassis_Path_NotifyTaskDone((uint8_t)'A');
        (void)Chassis_Path_SelectById(1U);
        (void)Chassis_Path_Start();
        mission_enter(MISSION_STATE_TO_B);
        break;

    default:
        break;
    }
}

static void mission_run_b(uint32_t now)
{
    switch (s_sub)
    {
    case 0U:
        (void)Arm_Release();
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 1U;
        break;

    case 1U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 2U;
        break;

    case 2U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_B, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 3U;
        break;

    case 3U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_B_OBJECT, 50U);
        s_sub_ms = now;
        s_t_ms = now + 100U;
        s_sub = 4U;
        break;

    case 4U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        {
            int r = mission_track_step(VISION_TRACK_B_OBJECT, MISSION_TRACK_TIMEOUT_MS);
            if (r > 0)
            {
                s_sub = 5U;
            }
            else if (r < 0)
            {
                if (s_track_retry < MISSION_TRACK_RETRY)
                {
                    s_track_retry++;
                    (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
                    s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
                    s_sub = 9U;   /* 回到 case 9 前的重粗调入口 */
                }
                else
                {
                    s_sub = 5U;   /* 放弃追踪，直接尝试抓取 */
                }
            }
            else
            {
                s_t_ms = now + 100U;
            }
        }
        break;

    case 9U:  /* 重粗调：回展开后重新预存并重发追踪 */
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_B, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 10U;
        break;

    case 10U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_B_OBJECT, 50U);
        s_sub_ms = now;
        s_t_ms = now + 100U;
        s_sub = 4U;
        break;

    case 5U:  /* 下降抓取 */
        if (Arm_IsBusy() != 0U) break;
        (void)Arm_ApproachTool(30, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 6U;
        break;

    case 6U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_Grasp();
        s_t_ms = now + MISSION_GRIP_MS;
        s_sub = 7U;
        break;

    case 7U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_ApproachTool(-30, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 8U;
        break;

    case 8U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 11U;
        break;

    case 11U: /* 底盘到桶 */
        if (s_servo_busy != 0U) break;
        if ((int32_t)(now - s_t_ms) < 0) break;
        {
            int16_t fwd = mission_bucket_offset_mm();
            if (fwd == 0)
            {
                s_t_ms = now;
                s_sub = 12U;
            }
            else if (Chassis_Path_ServoStep(fwd, 0) == 0)
            {
                s_servo_busy = 1U;
            }
            else
            {
                s_t_ms = now + 100U;   /* 重试 */
            }
        }
        break;

    case 12U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(-180.0f);
        s_t_ms = now + MISSION_J1_180_MS + MISSION_GAP_MS;
        s_sub = 13U;
        break;

    case 13U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_B_BUCKET, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 14U;
        break;

    case 14U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_B_BUCKET, 50U);
        s_sub_ms = now;
        s_t_ms = now + 100U;
        s_sub = 15U;
        break;

    case 15U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        {
            int r = mission_track_step(VISION_TRACK_B_BUCKET, MISSION_TRACK_TIMEOUT_MS);
            if (r != 0)
            {
                s_sub = 16U;   /* 失败也按名义桶位放置 */
            }
            else
            {
                s_t_ms = now + 100U;
            }
        }
        break;

    case 16U:
        if (Arm_IsBusy() != 0U) break;
        (void)Arm_ApproachTool(20, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 17U;
        break;

    case 17U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_Release();
        s_t_ms = now + MISSION_GRIP_MS;
        s_sub = 18U;
        break;

    case 18U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_ApproachTool(-20, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 19U;
        break;

    case 19U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 20U;
        break;

    case 20U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 21U;
        break;

    case 21U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 22U;
        break;

    case 22U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Chassis_Path_SelectById(2U);
        (void)Chassis_Path_Start();
        mission_enter(MISSION_STATE_TO_C);
        break;

    default:
        break;
    }
}

static void mission_run_c(uint32_t now)
{
    switch (s_sub)
    {
    case 0U:
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 1U;
        break;

    case 1U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 2U;
        break;

    case 2U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_C, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 3U;
        break;

    case 3U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_C, 50U);
        s_sub_ms = now;
        s_t_ms = now + 200U;
        s_conv = 0U;
        s_sub = 4U;
        break;

    case 4U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        if (s_servo_busy != 0U) break;
        {
            Vision_TrackSample_t ts;
            if (Vision_Link_GetTrack(VISION_TRACK_C, &ts) != 0U && ts.valid != 0U)
            {
                if (abs((int)ts.dy_mm) <= MISSION_SERVO_TOL_MM)
                {
                    s_conv++;
                }
                else
                {
                    s_conv = 0U;
                    {
                        int16_t fwd = (int16_t)(MISSION_C_SERVO_K * (float)ts.dy_mm);
                        if (fwd > MISSION_SERVO_STEP_MAX_MM)  fwd = MISSION_SERVO_STEP_MAX_MM;
                        if (fwd < -MISSION_SERVO_STEP_MAX_MM) fwd = -MISSION_SERVO_STEP_MAX_MM;
                        if (fwd != 0 && Chassis_Path_ServoStep(fwd, 0) == 0)
                        {
                            s_servo_busy = 1U;
                        }
                    }
                }

                if (s_conv >= MISSION_TRACK_CONV_SAMPLES
                    || (HAL_GetTick() - s_sub_ms) > MISSION_TRACK_TIMEOUT_MS)
                {
                    (void)Vision_Link_TrackStop(VISION_TRACK_C);
                    s_sub = 5U;
                }
            }
            s_t_ms = now + 200U;
        }
        break;

    case 5U:
        s_target_down = 0U;
        (void)Arm_Laser(1U);
        s_t_ms = now + MISSION_LASER_MS;
        s_sub = 6U;
        break;

    case 6U:
        if (s_target_down == 0U && (int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_Laser(0U);
        s_t_ms = now;
        s_sub = 7U;
        break;

    case 7U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 8U;
        break;

    case 8U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(-90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 9U;
        break;

    case 9U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 10U;
        break;

    case 10U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Chassis_Path_SelectById(3U);
        (void)Chassis_Path_Start();
        mission_enter(MISSION_STATE_TO_D);
        break;

    default:
        break;
    }
}

static void mission_run_d(uint32_t now)
{
    switch (s_sub)
    {
    case 0U:
        (void)Arm_Release();
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 1U;
        break;

    case 1U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 2U;
        break;

    case 2U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_D, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 3U;
        break;

    case 3U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_D, 50U);
        s_sub_ms = now;
        s_t_ms = now + 100U;
        s_sub = 4U;
        break;

    case 4U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        {
            int r = mission_track_step(VISION_TRACK_D, MISSION_TRACK_TIMEOUT_MS);
            if (r > 0)
            {
                s_sub = 5U;
            }
            else if (r < 0)
            {
                if (s_track_retry < MISSION_TRACK_RETRY)
                {
                    s_track_retry++;
                    (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
                    s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
                    s_sub = 9U;
                }
                else
                {
                    s_sub = 5U;
                }
            }
            else
            {
                s_t_ms = now + 100U;
            }
        }
        break;

    case 9U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_D, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 10U;
        break;

    case 10U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Vision_Link_TrackStart(VISION_TRACK_D, 50U);
        s_sub_ms = now;
        s_t_ms = now + 100U;
        s_sub = 4U;
        break;

    case 5U:
        if (Arm_IsBusy() != 0U) break;
        (void)Arm_ApproachTool(30, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 6U;
        break;

    case 6U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_Grasp();
        s_t_ms = now + MISSION_GRIP_MS;
        s_sub = 7U;
        break;

    case 7U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_ApproachTool(-30, 2000U);
        s_t_ms = now + MISSION_ARM_MS;
        s_sub = 8U;
        break;

    case 8U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_COMMON, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 11U;
        break;

    case 11U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MoveBaseRelative(-90.0f);
        s_t_ms = now + MISSION_J1_90_MS + MISSION_GAP_MS;
        s_sub = 12U;
        break;

    case 12U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
        s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
        s_sub = 13U;
        break;

    case 13U:
        if ((int32_t)(now - s_t_ms) < 0) break;
        (void)Chassis_Path_SelectById(4U);
        (void)Chassis_Path_Start();
        mission_enter(MISSION_STATE_RETURN);
        break;

    default:
        break;
    }
}

/* ===================== 对外接口 ===================== */

void Mission_Init(void)
{
    s_state = MISSION_STATE_IDLE;
    s_sub = 0U;
    s_degraded = 0U;
    s_abort_req = 0U;
    s_path_done = 0U;
    s_scan_result = 0U;
    s_rank = 1U;
    s_arrive_id = 0U;
    s_link_down = 0U;
    s_servo_busy = 0U;
}

int Mission_Start(void)
{
    if (s_state != MISSION_STATE_IDLE)
    {
        return -1;
    }

    s_degraded = 0U;
    s_fail = 0U;
    s_rank = 1U;
    s_path_done = 0U;
    s_arrive_id = 0U;
    s_abort_req = 0U;

    Chassis_Path_SetRank(1U);
    mission_enter(MISSION_STATE_PREP);
    return 0;
}

void Mission_RequestAbort(void)
{
    s_abort_req = 1U;
}

uint8_t Mission_IsActive(void)
{
    return (s_state != MISSION_STATE_IDLE
            && s_state != MISSION_STATE_DONE
            && s_state != MISSION_STATE_ERROR
            && s_state != MISSION_STATE_ABORT) ? 1U : 0U;
}

Mission_State_e Mission_GetState(void)
{
    return s_state;
}

void Mission_Update(void)
{
    uint32_t now = HAL_GetTick();

    mission_pump();

    if (s_abort_req != 0U)
    {
        s_abort_req = 0U;
        mission_hard_stop();
        mission_enter(MISSION_STATE_ABORT);
        return;
    }

    /* 失联超过 5 s：回正 + 收起 + 降级跑图 */
    if (s_link_down != 0U
        && (now - s_link_ms) > 5000U
        && s_state != MISSION_STATE_DEGRADED
        && s_state != MISSION_STATE_DONE
        && s_state != MISSION_STATE_ERROR)
    {
        float q[4];

        /* 先回正 J1，再收起机械臂，然后降级跑图 */
        if (Arm_GetJoints(q) == ARM_OK)
        {
            (void)Arm_MoveBaseRelative(-q[0]);
        }
        (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
        s_degraded = 1U;
        mission_enter(MISSION_STATE_DEGRADED);
    }

    switch (s_state)
    {
    case MISSION_STATE_IDLE:
    case MISSION_STATE_DONE:
    case MISSION_STATE_ERROR:
    case MISSION_STATE_ABORT:
        break;

    case MISSION_STATE_PREP:
        if (s_sub == 0U)
        {
            Arm_Status_e rc;

            (void)Arm_Release();
            rc = Arm_MoveJoint(A_Q, 8000U);

            if (rc == ARM_BUSY)
            {
                break;   /* 回读未就绪，下一拍重试 */
            }

            (void)Chassis_Path_SelectById(0U);
            (void)Chassis_Path_Start();
            s_t_ms = now + MISSION_ARM_MS + MISSION_GAP_MS;
            s_sub = 1U;
        }
        else if ((int32_t)(now - s_t_ms) >= 0)
        {
            mission_enter(MISSION_STATE_A_TRAVEL);
        }
        break;

    case MISSION_STATE_A_TRAVEL:
        if (s_arrive_id == (uint8_t)'A')
        {
            s_arrive_id = 0U;
            mission_enter(MISSION_STATE_A_TASK);
        }
        else if ((now - s_state_ms) > MISSION_A_TRAVEL_MS)
        {
            mission_hard_stop();
            mission_enter(MISSION_STATE_ERROR);
        }
        break;

    case MISSION_STATE_A_TASK:
        mission_run_a(now);
        break;

    case MISSION_STATE_TO_B:
        if (s_arrive_id == (uint8_t)'B')
        {
            s_arrive_id = 0U;
            s_path_done = 0U;
            (void)Vision_Link_CancelPending();
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'B');
            mission_enter(MISSION_STATE_B_TASK);
        }
        else if (s_path_done != 0U)
        {
            s_path_done = 0U;
            (void)Chassis_Path_SetRank(1U);
            mission_enter(MISSION_STATE_B_TASK);
        }
        break;

    case MISSION_STATE_B_TASK:
        if ((now - s_state_ms) > MISSION_B_TASK_MS)
        {
            (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
            (void)Chassis_Path_SelectById(2U);
            (void)Chassis_Path_Start();
            mission_enter(MISSION_STATE_TO_C);
            break;
        }
        mission_run_b(now);
        break;

    case MISSION_STATE_TO_C:
        if (s_arrive_id == (uint8_t)'C')
        {
            s_arrive_id = 0U;
            s_path_done = 0U;
            (void)Vision_Link_CancelPending();
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'C');
            mission_enter(MISSION_STATE_C_TASK);
        }
        break;

    case MISSION_STATE_C_TASK:
        if ((now - s_state_ms) > MISSION_C_TASK_MS)
        {
            (void)Arm_Laser(0U);
            (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
            (void)Chassis_Path_SelectById(3U);
            (void)Chassis_Path_Start();
            mission_enter(MISSION_STATE_TO_D);
            break;
        }
        mission_run_c(now);
        break;

    case MISSION_STATE_TO_D:
        if (s_arrive_id == (uint8_t)'D')
        {
            s_arrive_id = 0U;
            s_path_done = 0U;
            (void)Vision_Link_CancelPending();
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'D');
            mission_enter(MISSION_STATE_D_TASK);
        }
        break;

    case MISSION_STATE_D_TASK:
        if ((now - s_state_ms) > MISSION_D_TASK_MS)
        {
            (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
            (void)Chassis_Path_SelectById(4U);
            (void)Chassis_Path_Start();
            mission_enter(MISSION_STATE_RETURN);
            break;
        }
        mission_run_d(now);
        break;

    case MISSION_STATE_RETURN:
        if (s_path_done != 0U)
        {
            s_path_done = 0U;
            mission_enter(MISSION_STATE_DONE);
        }
        else if ((now - s_state_ms) > MISSION_RETURN_MS)
        {
            mission_hard_stop();
            mission_enter(MISSION_STATE_ERROR);
        }
        break;

    case MISSION_STATE_DEGRADED:
        /* 依次跑完全图，ARRIVE 当拍放行 */
        if (s_arrive_id != 0U)
        {
            uint8_t pid = s_arrive_id;

            s_arrive_id = 0U;
            (void)Chassis_Path_NotifyTaskDone(pid);
        }
        else if (s_path_done != 0U)
        {
            s_path_done = 0U;
            if (s_sub == 0U)
            {
                (void)Chassis_Path_SelectById(1U);
                (void)Chassis_Path_Start();
                s_sub = 1U;
            }
            else if (s_sub == 1U)
            {
                (void)Chassis_Path_SelectById(2U);
                (void)Chassis_Path_Start();
                s_sub = 2U;
            }
            else if (s_sub == 2U)
            {
                (void)Chassis_Path_SelectById(3U);
                (void)Chassis_Path_Start();
                s_sub = 3U;
            }
            else if (s_sub == 3U)
            {
                (void)Chassis_Path_SelectById(4U);
                (void)Chassis_Path_Start();
                s_sub = 4U;
            }
            else
            {
                mission_enter(MISSION_STATE_DONE);
            }
        }
        break;

    default:
        break;
    }
}
