/**
  ******************************************************************************
  * @file    chassis/planning/path.c
  * @brief   路径段调度（非阻塞状态机）。
  *
  * Chassis_Path_Update() 每 5ms 推进一次并立即返回，不做任何等待。
  * 段完成通过轮询控制层状态获得，任务点等待通过外部通知解除。
  ******************************************************************************
  */

#include "chassis/planning/path.h"
#include "chassis/planning/path_table.h"
#include "chassis/control/ctrl.h"
#include "chassis/chassis_config.h"
#include "chassis/chassis.h"
#include "common/math_util.h"
#include "main.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

typedef enum
{
    SEG_IDLE = 0,
    SEG_ACTIVE,      /* 控制层正在执行本段 */
    SEG_DWELL,       /* STOP 段计时 */
    SEG_SCAN_HOLD,   /* 扫描段走完，停车等待视觉 */
    SEG_SCAN_BACK,   /* 倒回扫描段中心 */
    SEG_CALIB,       /* 陀螺仪零偏重标定中 */
    SEG_TASK_WAIT    /* 等待外部任务完成 */
} Path_SegState_e;

static const Path_Table_t *s_table;
static volatile Plan_State_e s_state;
static volatile uint16_t s_index;
static uint16_t s_total;
static volatile uint8_t s_abort;

static Path_SegState_e s_seg_state;
static uint16_t s_seg_seq;
static uint16_t s_seq_counter;
static uint32_t s_seg_start_ms;
static uint32_t s_seg_timeout_ms;
static float    s_scan_dist;
static uint32_t s_scan_hold_until;

#define PATH_EVQ_LEN  (4U)
static volatile Path_Event_t s_evq[PATH_EVQ_LEN];
static volatile uint8_t s_evq_head;
static volatile uint8_t s_evq_tail;

static volatile uint8_t s_task_done_id;
static uint8_t s_rank = 1U;

/* 路径空闲时的单步伺服 */
static uint8_t  s_servo_active;
static uint16_t s_servo_seq;
static uint32_t s_servo_start_ms;
static uint32_t s_servo_timeout_ms;

static float seg_speed(const Path_Seg_t *seg)
{
    if (seg->v_mmps > 0.0f)
    {
        return seg->v_mmps;
    }
    return CHASSIS_V_CRUISE_MMPS;
}

static uint32_t seg_estimate_ms(const Path_Seg_t *seg)
{
    float t_ms;

    switch (seg->type)
    {
    case PATH_STRAIGHT:
    case PATH_SLOW:
    case PATH_SCAN:
        t_ms = seg->dist_mm / seg_speed(seg) * 1000.0f;
        break;

    case PATH_SPIN:
    {
        float w = CHASSIS_SPIN_W_MAX_DPS * 0.5f;
        t_ms = fabsf(seg->angle_deg) / w * 1000.0f;
        break;
    }

    case PATH_ARC:
    {
        float arc = fabsf(seg->angle_deg) * MATH_PI / 180.0f
                  * fabsf(seg->radius_mm);
        t_ms = arc / seg_speed(seg) * 1000.0f;
        break;
    }

    case PATH_STOP:
        t_ms = (float)seg->dwell_ms;
        break;

    default:
        t_ms = 0.0f;
        break;
    }

    t_ms *= CHASSIS_PLAN_TIMEOUT_FACTOR;

    if (t_ms < 2000.0f)
    {
        t_ms = 2000.0f;
    }

    return (uint32_t)t_ms;
}

static void path_emit(Path_Event_e type, uint8_t point_id, float value)
{
    uint8_t next = (uint8_t)((s_evq_head + 1U) % PATH_EVQ_LEN);

    if (next == s_evq_tail)
    {
        /* 队列满：丢弃最旧事件 */
        s_evq_tail = (uint8_t)((s_evq_tail + 1U) % PATH_EVQ_LEN);
    }

    s_evq[s_evq_head].type = type;
    s_evq[s_evq_head].point_id = point_id;
    s_evq[s_evq_head].value = value;
    s_evq_head = next;
}

static void path_error(void)
{
    Chassis_Ctrl_Stop(1U);
    s_state = PLAN_ERROR;
    s_seg_state = SEG_IDLE;
    path_emit(PATH_EV_ERROR, 0U, 0.0f);
}

static void path_advance(void)
{
    s_index++;

    if (s_index >= s_total)
    {
        s_state = PLAN_DONE;
        s_seg_state = SEG_IDLE;
        path_emit(PATH_EV_DONE, 0U, 0.0f);
        return;
    }

    s_seg_state = SEG_IDLE;
}

/* 下一段的入口速度：决定本段是否需要减速到 0。
   只有停车类段（任务点/停顿/原地转/路径结束）才需要归零。 */
static float path_next_entry_speed(uint16_t from_index)
{
    uint16_t i = from_index + 1U;
    const Path_Seg_t *next;

    if (i >= s_total)
    {
        return 0.0f;
    }

    next = &s_table->segs[i];

    switch (next->type)
    {
    case PATH_STRAIGHT:
    case PATH_SLOW:
    case PATH_ARC:
        return seg_speed(next);

    case PATH_SCAN:
        return (next->v_mmps > 0.0f) ? next->v_mmps : 100.0f;

    case PATH_TASK:
    case PATH_STOP:
    case PATH_SPIN:
    default:
        return 0.0f;
    }
}

/* 下发普通运动段（直行/弧线/旋转/低速） */
static void path_submit_motion(const Path_Seg_t *seg)
{
    Chassis_Cmd_t cmd;
    float v_mag = seg_speed(seg);

    memset(&cmd, 0, sizeof(cmd));

    switch (seg->type)
    {
    case PATH_STRAIGHT:
    case PATH_SLOW:
    case PATH_SCAN:   /* 扫描段先按直行执行，完成后由状态机接管倒回 */
        cmd.mode = CHASSIS_MODE_STRAIGHT;
        cmd.v_mmps = (seg->dir >= 0) ? v_mag : -v_mag;
        cmd.dist_mm = (seg->dir >= 0) ? seg->dist_mm : -seg->dist_mm;
        break;

    case PATH_SPIN:
        cmd.mode = CHASSIS_MODE_SPIN;
        cmd.v_mmps = 0.0f;
        cmd.heading_delta_deg = (seg->dir >= 0)
                              ? seg->angle_deg : -seg->angle_deg;
        break;

    case PATH_ARC:
        cmd.mode = CHASSIS_MODE_ARC;
        cmd.v_mmps = (seg->dir >= 0) ? v_mag : -v_mag;
        cmd.radius_mm = (seg->dir >= 0)
                      ? seg->radius_mm : -seg->radius_mm;
        cmd.heading_delta_deg = (seg->dir >= 0)
                              ? seg->angle_deg : -seg->angle_deg;
        break;

    default:
        return;
    }

    cmd.end_speed_mmps = path_next_entry_speed(s_index);
    cmd.brake_pulse = seg->brake;
    cmd.seg_last = (s_index == (uint16_t)(s_total - 1U)) ? 1U : 0U;
    cmd.seq = ++s_seq_counter;
    s_seg_seq = cmd.seq;

    Chassis_Ctrl_Submit(&cmd, 0U);

    s_seg_start_ms = HAL_GetTick();
    s_seg_timeout_ms = (seg->timeout_ms > 0U)
                     ? (uint32_t)seg->timeout_ms : seg_estimate_ms(seg);
    s_seg_state = SEG_ACTIVE;
}

/* 扫描段倒回中心：反向直行本段实际距离的一半 */
static void path_start_scan_back(const Path_Seg_t *seg)
{
    Chassis_Cmd_t cmd;
    float back_mm = s_scan_dist * 0.5f;

    /* rank 决定一次性倒回距离：0 多退 150，2 少退 150 */
    if (s_rank == 0U)
    {
        back_mm += 150.0f;
    }
    else if (s_rank == 2U)
    {
        back_mm -= 150.0f;
    }
    if (back_mm < 0.0f)
    {
        back_mm = 0.0f;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.mode = CHASSIS_MODE_STRAIGHT;
    cmd.v_mmps = -CHASSIS_SCAN_BACK_MMPS;   /* 回退用常速，与扫描慢行速度无关 */
    cmd.dist_mm = -back_mm;
    cmd.brake_pulse = 1U;
    cmd.seg_last = 0U;
    cmd.seq = ++s_seq_counter;
    s_seg_seq = cmd.seq;

    /* 反向行驶必须清除速度环积分：扫描段是正转，残留积分会与负目标冲突 */
    Chassis_Ctrl_Submit(&cmd, 1U);

    s_seg_start_ms = HAL_GetTick();
    s_seg_timeout_ms = seg_estimate_ms(seg);
    s_seg_state = SEG_SCAN_BACK;
}

static void path_start_segment(void)
{
    const Path_Seg_t *seg = &s_table->segs[s_index];

    switch (seg->type)
    {
    case PATH_STOP:
        s_seg_start_ms = HAL_GetTick();
        s_seg_timeout_ms = (uint32_t)seg->dwell_ms;
        s_seg_state = SEG_DWELL;
        break;

    case PATH_CALIB:
        /* 停车后在原地重新标定陀螺仪零偏，消除长时间运行的累积误差 */
        Chassis_Ctrl_Stop(1U);
        Chassis_CalibrateStart();
        s_seg_start_ms = HAL_GetTick();
        s_seg_timeout_ms = 5000U;
        s_seg_state = SEG_CALIB;
        break;

    case PATH_TASK:
        Chassis_Ctrl_Stop(1U);
        path_emit(PATH_EV_ARRIVE_POINT, seg->point_id, 0.0f);
        s_state = PLAN_WAIT_TASK;
        s_seg_start_ms = HAL_GetTick();
        s_seg_timeout_ms = seg->timeout_ms;
        s_seg_state = SEG_TASK_WAIT;
        break;

    case PATH_SCAN:
        path_emit(PATH_EV_SCAN_BEGIN, seg->point_id, 0.0f);
        path_submit_motion(seg);
        break;

    case PATH_STRAIGHT:
    case PATH_SLOW:
    case PATH_SPIN:
    case PATH_ARC:
        path_submit_motion(seg);
        break;

    default:
        path_advance();
        break;
    }
}

static void path_on_segment_done(const Chassis_Status_t *st)
{
    const Path_Seg_t *seg = &s_table->segs[s_index];

    if (seg->type == PATH_SCAN)
    {
        s_scan_dist = st->dist_done_mm;
        path_emit(PATH_EV_SCAN_READY, seg->point_id, s_scan_dist);

        if (seg->dwell_ms > 0U)
        {
            s_scan_hold_until = HAL_GetTick() + seg->dwell_ms;
            s_seg_state = SEG_SCAN_HOLD;
        }
        else
        {
            path_start_scan_back(seg);
        }
        return;
    }

    path_advance();
}

int Chassis_Path_Init(void)
{
    s_table = NULL;
    s_state = PLAN_IDLE;
    s_index = 0U;
    s_total = 0U;
    s_abort = 0U;
    s_seg_state = SEG_IDLE;
    s_seq_counter = 0U;
    s_task_done_id = 0U;
    s_evq_head = 0U;
    s_evq_tail = 0U;
    s_servo_active = 0U;
    s_rank = 1U;

    if (g_chassis_path_count > 0U)
    {
        (void)Chassis_Path_SelectById(0U);
    }

    return 0;
}

int Chassis_Path_SelectById(uint16_t id)
{
    if (id >= g_chassis_path_count)
    {
        return -1;
    }

    s_table = g_chassis_path_table[id];
    s_total = (s_table != NULL) ? s_table->count : 0U;
    s_state = PLAN_IDLE;
    s_seg_state = SEG_IDLE;

    return 0;
}

int Chassis_Path_SelectByName(const char *name)
{
    uint16_t i;

    if (name == NULL)
    {
        return -1;
    }

    for (i = 0; i < g_chassis_path_count; i++)
    {
        if (g_chassis_path_table[i] != NULL
            && strcmp(g_chassis_path_table[i]->name, name) == 0)
        {
            s_table = g_chassis_path_table[i];
            s_total = s_table->count;
            s_state = PLAN_IDLE;
            s_seg_state = SEG_IDLE;
            return 0;
        }
    }

    return -1;
}

int Chassis_Path_Start(void)
{
    if (s_table == NULL || s_table->count == 0U)
    {
        return -1;
    }

    s_state = PLAN_RUNNING;
    s_index = 0U;
    s_abort = 0U;
    s_seg_state = SEG_IDLE;
    s_seq_counter = 0U;
    s_task_done_id = 0U;
    s_servo_active = 0U;

    Chassis_Ctrl_BeginPath();

    return 0;
}

void Chassis_Path_Abort(void)
{
    s_abort = 1U;
    Chassis_Ctrl_Stop(1U);
    s_state = PLAN_IDLE;
    s_seg_state = SEG_IDLE;
}

Plan_State_e Chassis_Path_GetState(uint16_t *seg_idx, uint16_t *seg_total)
{
    if (seg_idx != NULL)
    {
        *seg_idx = s_index;
    }
    if (seg_total != NULL)
    {
        *seg_total = s_total;
    }
    return s_state;
}

uint8_t Chassis_Path_IsRunning(void)
{
    return (s_state == PLAN_RUNNING) ? 1U : 0U;
}

uint8_t Chassis_Path_IsWaitingTask(void)
{
    return (s_state == PLAN_WAIT_TASK) ? 1U : 0U;
}

void Chassis_Path_Update(void)
{
    Chassis_Status_t st;
    const Path_Seg_t *seg;

    if (s_abort != 0U)
    {
        Chassis_Ctrl_Stop(1U);
        s_state = PLAN_IDLE;
        s_seg_state = SEG_IDLE;
        s_abort = 0U;
        return;
    }

    /* 路径空闲时的单步伺服：完成后发 AUX_DONE */
    if (s_servo_active != 0U)
    {
        Chassis_Ctrl_GetStatus(&st);

        if ((st.finished != 0U && st.seq == s_servo_seq)
            || (HAL_GetTick() - s_servo_start_ms) > s_servo_timeout_ms)
        {
            s_servo_active = 0U;
            path_emit(PATH_EV_AUX_DONE, 0U, 0.0f);
        }
        return;
    }

    if (s_state != PLAN_RUNNING && s_state != PLAN_WAIT_TASK)
    {
        return;
    }

    switch (s_seg_state)
    {
    case SEG_IDLE:
        path_start_segment();
        break;

    case SEG_ACTIVE:
    case SEG_SCAN_BACK:
        if (s_seg_timeout_ms > 0U
            && (HAL_GetTick() - s_seg_start_ms) > s_seg_timeout_ms)
        {
            path_error();
            break;
        }

        Chassis_Ctrl_GetStatus(&st);

        if (st.fault != 0U && st.seq == s_seg_seq)
        {
            path_error();
            break;
        }

        if (st.finished != 0U && st.seq == s_seg_seq)
        {
            if (s_seg_state == SEG_SCAN_BACK)
            {
                seg = &s_table->segs[s_index];
                path_emit(PATH_EV_ARRIVE_POINT, seg->point_id, 0.0f);
                s_state = PLAN_WAIT_TASK;
                s_seg_state = SEG_TASK_WAIT;
            }
            else
            {
                path_on_segment_done(&st);
            }
        }
        break;

    case SEG_DWELL:
        if ((HAL_GetTick() - s_seg_start_ms) >= s_seg_timeout_ms)
        {
            path_advance();
        }
        break;

    case SEG_SCAN_HOLD:
        if ((int32_t)(HAL_GetTick() - s_scan_hold_until) >= 0)
        {
            path_start_scan_back(&s_table->segs[s_index]);
        }
        break;

    case SEG_CALIB:
        if (Chassis_CalibratePoll() != 0U)
        {
            path_advance();
        }
        else if ((HAL_GetTick() - s_seg_start_ms) > s_seg_timeout_ms)
        {
            path_error();
        }
        break;

    case SEG_TASK_WAIT:
        seg = &s_table->segs[s_index];
        if (s_task_done_id != 0U && s_task_done_id == seg->point_id)
        {
            s_task_done_id = 0U;
            s_state = PLAN_RUNNING;
            path_advance();
        }
        else if (s_seg_timeout_ms > 0U
                 && (HAL_GetTick() - s_seg_start_ms) > s_seg_timeout_ms)
        {
            path_error();
        }
        break;

    default:
        s_seg_state = SEG_IDLE;
        break;
    }
}

uint8_t Chassis_Path_TakeEvent(Path_Event_t *ev)
{
    if (ev == NULL || s_evq_tail == s_evq_head)
    {
        return 0U;
    }

    *ev = s_evq[s_evq_tail];
    s_evq_tail = (uint8_t)((s_evq_tail + 1U) % PATH_EVQ_LEN);
    return 1U;
}

int Chassis_Path_NotifyTaskDone(uint8_t point_id)
{
    if (s_state != PLAN_WAIT_TASK || s_table == NULL)
    {
        return -1;
    }

    if (s_table->segs[s_index].point_id != point_id)
    {
        return -1;
    }

    s_task_done_id = point_id;
    return 0;
}

void Chassis_Path_SetRank(uint8_t rank)
{
    s_rank = (rank <= 2U) ? rank : 1U;
}

int Chassis_Path_ServoStep(int16_t forward_mm, int16_t turn_deg)
{
    Chassis_Cmd_t cmd;

    if (s_state == PLAN_RUNNING || s_state == PLAN_WAIT_TASK || s_servo_active != 0U)
    {
        return -1;
    }

    if (forward_mm == 0 && turn_deg == 0)
    {
        return -1;
    }

    memset(&cmd, 0, sizeof(cmd));

    if (turn_deg != 0)
    {
        cmd.mode = CHASSIS_MODE_SPIN;
        cmd.heading_delta_deg = (float)turn_deg;
    }
    else
    {
        cmd.mode = CHASSIS_MODE_STRAIGHT;
        cmd.v_mmps = (forward_mm >= 0) ? 120.0f : -120.0f;
        cmd.dist_mm = (float)forward_mm;
    }

    cmd.brake_pulse = 1U;
    cmd.seg_last = 0U;
    cmd.seq = ++s_seq_counter;

    s_servo_seq = cmd.seq;
    s_servo_start_ms = HAL_GetTick();
    s_servo_timeout_ms = (forward_mm != 0)
                       ? (uint32_t)(fabsf((float)forward_mm) / 120.0f * 1000.0f) + 2000U
                       : (uint32_t)(fabsf((float)turn_deg) / 60.0f * 1000.0f) + 2000U;
    s_servo_active = 1U;

    Chassis_Ctrl_Submit(&cmd, 1U);
    return 0;
}
