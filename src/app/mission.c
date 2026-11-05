/**
  ******************************************************************************
  * @file    app/mission.c
  * @brief   正式模式任务层：A/B/C/D 编排、视觉事件消费、失败降级。
  *
  * 与 Vision_Link_Update() 同任务上下文运行（MainTask / MissionTask）。
  * 路径事件与视觉事件的唯一消费者。
  ******************************************************************************
  */

#include "app/app_debug.h"
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
static uint8_t  s_order_ready;
static uint8_t  s_rank_moving;
static uint8_t  s_midline_got;
static uint8_t  s_midline_result;
static uint8_t  s_midline_last;
static int8_t   s_midline_dir;      /* -1=后退 0=停 +1=前进 */
static uint8_t  s_midline_no_resp;
static uint32_t s_midline_start_ms;
static uint32_t s_midline_next_ms;
static uint32_t s_midline_rx_count;
static uint32_t s_midline_rx_seen;
static int16_t  s_align_mm;        /* 首次对准基准之后累计的实际位移 */
static uint8_t  s_align_accum;     /* 下一条 AUX_DONE 是否计入 s_align_mm */
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
static uint8_t  s_stop_after_a;

static const uint16_t POSE_A_SCAN[3] __attribute__((unused)) = MISSION_POSE_A_SCAN;
static const uint16_t POSE_COMMON[3] = MISSION_POSE_COMMON_OPEN;
static const uint16_t POSE_B[3]      = MISSION_POSE_B_PRESET;
static const uint16_t POSE_B_BUCKET[3] = MISSION_POSE_B_BUCKET;
static const uint16_t POSE_C[3]      = MISSION_POSE_C_PRESET;
static const uint16_t POSE_D[3]      = MISSION_POSE_D_PRESET;
static const uint16_t POSE_STOW[3]   = MISSION_POSE_STOW;

/* A 扫码姿态关节角：PTP 备用，当前应急模式用 PWM 直发 */
static const float A_Q[4] __attribute__((unused)) = {
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
    s_order_ready = 0U;
    s_rank_moving = 0U;
    s_midline_got = 0U;
    s_midline_result = 0U;
    s_midline_last = 0U;
    s_midline_dir = 0;
    s_midline_no_resp = 0U;
    s_align_mm = 0;
    s_align_accum = 0U;
    s_servo_busy = 0U;
    s_servo_pending = 0U;
}

static void mission_hard_stop(void)
{
    Chassis_Path_Abort();
    Arm_Stop();
    (void)Arm_Laser(0U);
}

/* A 点失败：只到 A 模式直接结束，否则进入降级跑图 */
static void mission_a_fail(void)
{
    /* 扫码等待已结束：启动回收，不等待完成 */
    (void)Arm_Close();
    (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
    (void)Chassis_Path_NotifyTaskDone((uint8_t)'A');

    if (s_stop_after_a != 0U)
    {
        mission_enter(MISSION_STATE_DONE);
    }
    else
    {
        /* 回收与行驶并行：立即交给演示流程选表 1 继续跑 B/C/D */
        App_Debug_StartDemoFromA();
        mission_enter(MISSION_STATE_DEMO);
    }
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
            if (s_align_accum != 0U)
            {
                s_align_mm += (int16_t)pe.value;
                s_align_accum = 0U;
            }
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
            s_scan_result = 2U;
            break;

        case VISION_EV_SCAN_TIMEOUT:
            s_scan_result = 3U;
            break;

        case VISION_EV_MIDLINE:
            s_midline_result = ve.value;
            s_midline_got = 1U;
            s_midline_rx_count++;
            break;

        case VISION_EV_ORDER:
            s_rank = ve.value;
            if (s_rank > 2U)
            {
                s_rank = 1U;
            }
            s_order_ready = 1U;
            /* 兼容仍带扫描段的路径表；当前表已改为停车后按距离移动 */
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
/* 主换算仍用 rank ±150；中位线只做微调，不参与此处距离计算。 */
static int16_t mission_bucket_offset_mm(void)
{
    if (s_rank == 0U)
    {
        return 150;     /* 近点已多退 150，需前进 */
    }
    if (s_rank == 2U)
    {
        return -150;    /* 远点少退 150，需后退 */
    }
    return 0;
}

/* ===================== 任务点居中后的名次换算 ===================== */
/*
 * 停车点位于任务区中间（原扫描段中点）。rank0 是最近点、已被车驶过，
 * 需后退一个物品间距；rank2 是最远点，需前进一个间距；rank1 原地不动。
 */

static int16_t mission_rank_offset_mm(void)
{
    if (s_rank == 0U)
    {
        return -(int16_t)MISSION_RANK_DIST_MM;
    }
    if (s_rank == 2U)
    {
        return (int16_t)MISSION_RANK_DIST_MM;
    }
    return 0;
}

/* 返回 0=移动中/等待，1=已到名次对应位置 */
static int mission_rank_move_step(void)
{
    int16_t off;

    if (s_servo_busy != 0U)
    {
        return 0;
    }

    if (s_rank_moving != 0U)
    {
        s_rank_moving = 0U;
        return 1;
    }

    off = mission_rank_offset_mm();
    if (off == 0)
    {
        return 1;
    }

    if (Chassis_Path_ServoStep(off, 0) == 0)
    {
        s_servo_busy = 1U;
        s_rank_moving = 1U;
        s_align_accum = 1U;
    }

    return 0;
}

/* 向视觉请求本区域目标名次；请求失败按 rank=1 继续 */
static void mission_request_order(Vision_Zone_e zone, uint32_t now)
{
    s_order_ready = 0U;
    s_rank = 1U;

    if (Vision_Link_RequestOrder(zone) == VISION_LINK_OK)
    {
        s_t_ms = now + MISSION_ORDER_WAIT_MS;
    }
    else
    {
        s_order_ready = 1U;
        s_t_ms = now;
    }
}

/* ===================== 中位线对准 ===================== */
/*
 * 协议 0x21/0x31/0x41：0=未检出 1=已进入中位线范围 2=偏左 3=偏右。
 * 方向已确认：2（偏左）后退、3（偏右）前进。
 * 每 500 ms 发一帧请求，底盘按最近判定以 MISSION_MIDLINE_SPEED_MMPS 连续低速移动；
 * 5 s 总超时或连续 3 个窗口无响应即原地停车，继续任务。
 * 停止时路径层 AUX_DONE 回传实际位移，记入 s_align_mm 仅作记录；
 * 到桶等主换算仍使用 rank ±150，不受中位线微调影响。
 */

/* 让底盘以低速连续前后运动；dir: -1=后退 0=停 +1=前进
 *
 * 审计 H7：只有 Chassis_Path_ServoRun() 真正启动成功才把 dir 提交进 s_midline_dir。
 * 原实现先把目标方向写进 s_midline_dir 再去启动，启动失败（路径层 PLAN_RUNNING 或
 * 已有伺服段在跑时返回 -1）不回滚，下一拍开头的 if (dir == s_midline_dir) 直接放行，
 * 于是「逻辑在走、车不动」直到 5s 总超时。现在失败即保持未驱动状态，下一拍自然重试。 */
static void mission_midline_drive(int8_t dir)
{
    if (dir == s_midline_dir)
    {
        return;
    }

    if (s_midline_dir != 0)
    {
        /* 停止时路径层回传实际位移，记入 s_align_mm（仅记录，不参与到桶换算） */
        s_align_accum = 1U;
        Chassis_Path_ServoStop();
        s_midline_dir = 0;
    }

    if (dir == 0)
    {
        return;
    }

    {
        float v = (dir > 0) ? MISSION_MIDLINE_SPEED_MMPS
                            : -MISSION_MIDLINE_SPEED_MMPS;

        if (Chassis_Path_ServoRun(v) == 0)
        {
            s_midline_dir = dir;    /* 启动成功才提交目标方向 */
        }
        else
        {
            s_midline_dir = 0;      /* 启动失败：不记方向，下一拍重试 */
        }
    }
}

static void mission_midline_start(Vision_Zone_e zone, uint32_t now)
{
    s_midline_got = 0U;
    s_midline_result = 0U;
    s_midline_last = 0U;
    s_midline_dir = 0;
    s_midline_no_resp = 0U;
    s_midline_rx_count = 0U;
    s_midline_rx_seen = 0U;
    s_midline_start_ms = now;
    s_midline_next_ms = now + MISSION_MIDLINE_PERIOD_MS;

    (void)Vision_Link_RequestMidline(zone);
}

/* 返回 0=仍在对准，1=已对准或放弃 */
static int mission_midline_step(Vision_Zone_e zone, uint32_t now)
{
    /* 新结果立即生效：1 表示已进入中位线范围 */
    if (s_midline_got != 0U)
    {
        s_midline_got = 0U;
        s_midline_last = s_midline_result;

        if (s_midline_last == 1U)
        {
            mission_midline_drive(0);
            (void)Vision_Link_CancelMidline();
            return 1;
        }
    }

    /* 总超时 5 s：原地停车，开始任务 */
    if ((now - s_midline_start_ms) >= MISSION_MIDLINE_TOTAL_MS)
    {
        mission_midline_drive(0);
        (void)Vision_Link_CancelMidline();
        return 1;
    }

    /* 每 500 ms 一帧：统计无响应并重发 */
    if ((int32_t)(now - s_midline_next_ms) >= 0)
    {
        if (s_midline_rx_count == s_midline_rx_seen)
        {
            s_midline_no_resp++;
        }
        else
        {
            s_midline_no_resp = 0U;
            s_midline_rx_seen = s_midline_rx_count;
        }

        /* 连续 3 个窗口无响应：原地停车，开始任务 */
        if (s_midline_no_resp >= MISSION_MIDLINE_NO_RESP)
        {
            mission_midline_drive(0);
            (void)Vision_Link_CancelMidline();
            return 1;
        }

        (void)Vision_Link_CancelMidline();
        (void)Vision_Link_RequestMidline(zone);
        s_midline_next_ms = now + MISSION_MIDLINE_PERIOD_MS;
    }

    /* 按最近一次判定维持低速连续运动，对准后由上方的 1 停车 */
    if (s_midline_last == 2U)
    {
        mission_midline_drive(-1);
    }
    else if (s_midline_last == 3U)
    {
        mission_midline_drive(1);
    }
    else
    {
        mission_midline_drive(0);
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
            break;   /* 等腕部到位，不动 J1 */
        }
        /* 到达 A 点后先停稳，再发 0x10；3 s 计时从发送时刻开始 */
        if ((now - s_state_ms) < MISSION_A_SCAN_SETTLE_MS)
        {
            break;
        }
        s_scan_result = 0U;
        if (Vision_Link_RequestScanCode() != VISION_LINK_OK)
        {
            s_scan_result = 3U;   /* 发送失败按超时计一次 */
        }
        s_t_ms = now + MISSION_SCAN_CODE_TIMEOUT_MS;
        s_sub = 1U;
        break;

    case 1U:
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
            /* 扫到码：启动回收，同时放行 A 点并发车前往 B */
            (void)Arm_Close();
            (void)Arm_MovePwm(POSE_STOW, MISSION_ARM_MS);
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'A');

            if (s_stop_after_a != 0U)
            {
                mission_enter(MISSION_STATE_DONE);
            }
            else
            {
                (void)Chassis_Path_SelectById(1U);
                (void)Chassis_Path_Start();
                mission_enter(MISSION_STATE_TO_B);
            }
        }
        else
        {
            /* 0x10 发出 3 s 无响应或收到 0x12：回收 + 交给演示流程继续 */
            mission_a_fail();
        }
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
        /* 展开到位后开始扫描并做首次中位线对准 */
        (void)Vision_Link_StartScan(VISION_ZONE_B);
        mission_midline_start(VISION_ZONE_B, now);
        s_sub = 32U;
        break;

    case 32U: /* 首次中位线对准 */
        if (mission_midline_step(VISION_ZONE_B, now) == 0) break;
        /* 首次对准建立新的纵向基准，此前的移动不计入到桶补偿 */
        s_align_mm = 0;
        s_align_accum = 0U;
        mission_request_order(VISION_ZONE_B, now);
        s_sub = 30U;
        break;

    case 30U: /* 等视觉名次（3.5 s 兜底 rank=1） */
        if (s_order_ready == 0U && (int32_t)(now - s_t_ms) < 0) break;
        s_sub = 31U;
        break;

    case 31U: /* 按名次前后移动到目标球 */
        if (mission_rank_move_step() == 0) break;
        mission_midline_start(VISION_ZONE_B, now);   /* 到位后重新定位 */
        s_sub = 34U;
        break;

    case 34U: /* 名次到位后的中位线复核 */
        if (mission_midline_step(VISION_ZONE_B, now) == 0) break;
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
        /* 展开到位后开始扫描并做首次中位线对准 */
        (void)Vision_Link_StartScan(VISION_ZONE_C);
        mission_midline_start(VISION_ZONE_C, now);
        s_sub = 32U;
        break;

    case 32U: /* 首次中位线对准 */
        if (mission_midline_step(VISION_ZONE_C, now) == 0) break;
        mission_request_order(VISION_ZONE_C, now);
        s_sub = 30U;
        break;

    case 30U: /* 等视觉名次（3.5 s 兜底 rank=1） */
        if (s_order_ready == 0U && (int32_t)(now - s_t_ms) < 0) break;
        s_sub = 31U;
        break;

    case 31U: /* 按名次前后移动到目标靶 */
        if (mission_rank_move_step() == 0) break;
        mission_midline_start(VISION_ZONE_C, now);   /* 到位后重新定位 */
        s_sub = 34U;
        break;

    case 34U: /* 名次到位后的中位线复核 */
        if (mission_midline_step(VISION_ZONE_C, now) == 0) break;
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
        /* 展开到位后开始扫描并做首次中位线对准 */
        (void)Vision_Link_StartScan(VISION_ZONE_D);
        mission_midline_start(VISION_ZONE_D, now);
        s_sub = 32U;
        break;

    case 32U: /* 首次中位线对准 */
        if (mission_midline_step(VISION_ZONE_D, now) == 0) break;
        mission_request_order(VISION_ZONE_D, now);
        s_sub = 30U;
        break;

    case 30U: /* 等视觉名次（3.5 s 兜底 rank=1） */
        if (s_order_ready == 0U && (int32_t)(now - s_t_ms) < 0) break;
        s_sub = 31U;
        break;

    case 31U: /* 按名次前后移动到目标 */
        if (mission_rank_move_step() == 0) break;
        mission_midline_start(VISION_ZONE_D, now);   /* 到位后重新定位 */
        s_sub = 34U;
        break;

    case 34U: /* 名次到位后的中位线复核 */
        if (mission_midline_step(VISION_ZONE_D, now) == 0) break;
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

static int mission_start_common(uint8_t stop_after_a)
{
    if (s_state != MISSION_STATE_IDLE)
    {
        return -1;
    }

    s_degraded = 0U;
    s_rank = 1U;
    s_path_done = 0U;
    s_arrive_id = 0U;
    s_abort_req = 0U;
    s_stop_after_a = stop_after_a;

    Chassis_Path_SetRank(1U);
    mission_enter(MISSION_STATE_PREP);
    return 0;
}

int Mission_Start(void)
{
    return mission_start_common(0U);
}

int Mission_StartToA(void)
{
    return mission_start_common(1U);
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

uint8_t Mission_OwnsPath(void)
{
    return (App_Debug_IsDemoActive() != 0U) ? 0U : 1U;
}

void Mission_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 演示流程（扫码失败或模拟按键）独占路径与机械臂 */
    if (App_Debug_IsDemoActive() != 0U)
    {
        if (s_abort_req != 0U)
        {
            s_abort_req = 0U;
            App_Debug_StopDemo();
            mission_hard_stop();
            mission_enter(MISSION_STATE_ABORT);
        }
        return;
    }

    if (s_state == MISSION_STATE_DEMO)
    {
        mission_enter(MISSION_STATE_DONE);
        return;
    }

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
            /* A 扫码姿态：大臂小臂不动，腕部平伸（q4=0，PWM 1559） */
            (void)Arm_Release();
            (void)Arm_SetWristPwm(1559U, MISSION_ARM_MS);
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
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'B');
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
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'C');
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
            (void)Chassis_Path_NotifyTaskDone((uint8_t)'D');
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
