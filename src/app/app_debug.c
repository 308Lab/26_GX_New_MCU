/**
  ******************************************************************************
  * @file    app/app_debug.c
  * @brief   OLED 调试视图。自实现定点格式化，不依赖 stdio。
  ******************************************************************************
  */

#include "app/app_debug.h"
#include "app/mission.h"
#include "app/vision_link.h"
#include "arm/arm.h"
#include "arm/arm_debug.h"
#include "arm/arm_config.h"
#include "chassis/chassis.h"
#include "common/keypad.h"
#include "common/oled.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_LINE_MAX  (24U)

/* 单阶段调试：固定为 APP_DEBUG_FIXED_STEP，不做运行时切页。 */
static volatile App_Debug_Step_e s_debug_step = APP_DEBUG_FIXED_STEP;

/* S16 当前选中的舵机：0=J2, 1=J3, 2=J4 */
static uint8_t s_arm_axis;

/* 当前操作的舵机 ID（旧页面保留） */
static uint8_t s_id_target;
static uint8_t s_tq_on = 1U;
static uint8_t s_laser_on;

/* 按 2 触发的一轮回读：ID1 -> ID2 -> ID3 */
static uint8_t  s_read_active;
static uint8_t  s_read_done;
static uint8_t  s_read_id = 1U;
static uint32_t s_read_next_ms;

/* 动作调试页：待启动动作与重试 */
typedef enum
{
    ARM_PEND_NONE = 0,
    ARM_PEND_HOME,
    ARM_PEND_STOW,
    ARM_PEND_JOINT,
    ARM_PEND_POSE
} Arm_Pend_e;

static Arm_Pend_e   s_pending_action;
static uint32_t     s_pending_since;
static uint32_t     s_pending_next;
static Arm_Status_e s_action_error;

/* 扫描页状态（保留） */
static uint8_t s_show_scan;

/* S9 目标轮速，仅用于显示 */
static volatile float s_dbg_tl;
static volatile float s_dbg_tr;

/* S10 目标距离/速度，仅用于显示 */
static volatile float s_dbg_dist;
static volatile float s_dbg_speed;

/* S11 目标角度，仅用于显示 */
static volatile float s_dbg_angle;

/* S12 目标半径，仅用于显示 */
static volatile float s_dbg_radius;
static volatile uint8_t s_last_key;

static uint32_t s_rate_last_seq;
static uint32_t s_rate_last_tick;
static uint32_t s_frame_rate;

/* S3 陀螺诊断：按键 2 启动一次非阻塞零偏标定 */
static uint8_t s_s3_calib_active;

/* 上电陀螺零偏校准结果：0=进行中/未校准，1=OK，2=失败 */
static uint8_t s_gyro_calib_state;

/* S4 点动状态：按键只设定目标与到期时间，由 App_Debug_Update 非阻塞收尾 */
static int16_t  s_jog_l;
static int16_t  s_jog_r;
static uint32_t s_jog_end_ms;
static uint8_t  s_jog_active;

static const char *debug_step_name(App_Debug_Step_e step)
{
    switch (step)
    {
    case DEBUG_STEP_S1:  return "S1";
    case DEBUG_STEP_S3:  return "S3";
    case DEBUG_STEP_S4:  return "S4";
    case DEBUG_STEP_S5:  return "S5";
    case DEBUG_STEP_S6:  return "S6";
    case DEBUG_STEP_S7:  return "S7";
    case DEBUG_STEP_S8:  return "S8";
    case DEBUG_STEP_S9:  return "S9";
    case DEBUG_STEP_S10: return "SA";
    case DEBUG_STEP_S11: return "SB";
    case DEBUG_STEP_S12: return "SC";
    case DEBUG_STEP_S13: return "SD";
    case DEBUG_STEP_S14: return "SE";
    case DEBUG_STEP_S15: return "J1";
    case DEBUG_STEP_S16: return "SV";
    case DEBUG_STEP_S17: return "GL";
    case DEBUG_STEP_S18: return "PTP";
    case DEBUG_STEP_S19: return "ID";
    case DEBUG_STEP_S20: return "CAL";
    case DEBUG_STEP_S21: return "J3RP";
    default:             return "S?";
    }
}

void App_Debug_SetStep(App_Debug_Step_e step)
{
    s_debug_step = step;
}

App_Debug_Step_e App_Debug_GetStep(void)
{
    return s_debug_step;
}

uint8_t App_Debug_GetLastKey(void)
{
    return s_last_key;
}

void App_Debug_SetGyroCalib(uint8_t state)
{
    s_gyro_calib_state = state;
}

static void debug_start_jog(uint8_t key)
{
    switch (key)
    {
    case (uint8_t)'1': s_jog_l =  300; s_jog_r =    0; break;  /* 左轮正转 */
    case (uint8_t)'2': s_jog_l =    0; s_jog_r =  300; break;  /* 右轮正转 */
    case (uint8_t)'3': s_jog_l =  300; s_jog_r =  300; break;  /* 双轮正转 */
    case (uint8_t)'7': s_jog_l = -300; s_jog_r =    0; break;  /* 左轮反转 */
    case (uint8_t)'8': s_jog_l =    0; s_jog_r = -300; break;  /* 右轮反转 */
    case (uint8_t)'9': s_jog_l = -300; s_jog_r = -300; break;  /* 双轮反转 */
    default:
        return;
    }

    s_jog_end_ms = HAL_GetTick() + APP_DEBUG_JOG_MS;
    s_jog_active = 1U;
    Chassis_Debug_SetMotor(s_jog_l, s_jog_r);
}

/* 调试阶段的任务模拟：到达任务点后延时 APP_DEBUG_TASK_MS 自动放行。
   接入真实任务层后，改为由任务模块调用 Chassis_Path_NotifyTaskDone()。 */
static uint8_t  s_task_pending;
static uint32_t s_task_done_at;
static uint8_t  s_task_point;
static volatile Path_Event_e s_last_path_ev;

/* 整条路径串行执行（按键 9）：0→A→B→C→D→终点，每点停留 2s */
#define APP_DEBUG_PATH_TABLE_COUNT  (5U)
static uint8_t  s_chain_active;
static uint8_t  s_chain_index;
static uint8_t  s_chain_pending;
static uint32_t s_chain_next_ms;

/* S14 键 3：张爪 → 摆臂 → 到 A → J1 右转 90° → 扫码 */
static const float s_a_scan_q[4] = { 0.0f, 150.44f, 164.28f, 36.45f };
/* 应急模式：A 扫码姿态 PWM（J2/J3/J4），直接下发、不等回读 */
static const uint16_t s_a_scan_pwm[3] __attribute__((unused)) = { 1976U, 2127U, 1829U };
static uint8_t  s_a_arm_flow;      /* 1 = 流程激活 */
static uint8_t  s_a_arm_state;     /* 3=等夹爪张开，0=等路径DONE，1=等J1转向，2=已完成 */
static uint32_t s_a_gripper_ms;
static uint32_t s_a_j1_start_ms;
#define APP_A_GRIPPER_MS   (800U)   /* 夹爪张开时间 */
#define APP_A_J1_MOVE_MS   (4000U)  /* J1 转 90° 约 3.5 s，留余量 */

/* S14 键 1：到 A 后发一次扫码指令 */
static uint8_t  s_key1_scan;

/* S14 键 9：演示模式 A→B→C→D 全流程 */
static uint8_t  s_full_flow;
static uint8_t  s_full_state;
static uint32_t s_full_t_ms;
static uint32_t s_full_fb_ms;
static uint8_t  s_demo_read_id = 1U;

/* 演示模式机械臂位姿（PWM：J2/J3/J4） */
/* B/C/D 的机械臂位姿暂时保留，当前流程不调用 */
#define DEMO_POSE_UNUSED __attribute__((unused))
static const uint16_t s_pose_common[3] DEMO_POSE_UNUSED   = ARM_POSE_COMMON_OPEN_PWM;
static const uint16_t s_pose_b_demo[3] DEMO_POSE_UNUSED   = ARM_POSE_B_DEMO_PWM;
static const uint16_t s_pose_b_bucket[3] DEMO_POSE_UNUSED = ARM_POSE_B_BUCKET_DEMO_PWM;
static const uint16_t s_pose_c[3] DEMO_POSE_UNUSED        = ARM_POSE_C_PRESET_PWM;
static const uint16_t s_pose_d[3] DEMO_POSE_UNUSED        = ARM_POSE_D_DEMO_PWM;
/* 收起姿态 PWM：q2=181°, q3=178°, q4=78°（新 J3 标定） */
static const uint16_t s_pose_stow[3]     = { 2206U, 2114U, 2137U };

#define DEMO_ARM_MS        (3000U)  /* 机械臂每个 PWM 动作 3 s */
#define DEMO_GAP_MS        (2000U)  /* 机械臂与步进之间的间隔 2 s */
#define DEMO_GRIP_MS       (800U)   /* 夹爪动作等待 */
#define DEMO_J1_90_MS      (4000U)  /* J1 转 90° 约 3.5 s */
#define DEMO_J1_180_MS     (8000U)  /* J1 转 180° 约 7 s */
#define DEMO_LASER_MS      (2000U)  /* 演示模式激光 2 s */

void App_Debug_StartDemo(void)
{
    (void)Arm_Release();
    (void)Arm_SetWristPwm(1559U, DEMO_ARM_MS);

    s_full_flow = 1U;
    s_full_state = 1U;
    s_full_t_ms = HAL_GetTick() + DEMO_ARM_MS + DEMO_GAP_MS;
    s_full_fb_ms = HAL_GetTick();

    (void)Chassis_Path_SelectById(0U);
    (void)Chassis_Path_Start();
}

void App_Debug_StartDemoFromA(void)
{
    /* A 点回收已由 mission 启动，这里直接选表 1 继续演示 */
    s_full_flow = 1U;
    s_full_state = 5U;
    s_full_t_ms = HAL_GetTick();
    s_full_fb_ms = HAL_GetTick();
}

void App_Debug_StopDemo(void)
{
    s_full_flow = 0U;
    s_full_state = 39U;
}

uint8_t App_Debug_IsDemoActive(void)
{
    return s_full_flow;
}

void App_Debug_HandlePathEvents(void)
{
    Path_Event_t ev;

    while (Chassis_Path_TakeEvent(&ev))
    {
        s_last_path_ev = ev.type;

        if (ev.type == PATH_EV_ARRIVE_POINT)
        {
            if (s_full_flow != 0U && ev.point_id == (uint8_t)'A')
            {
                /* A 点：立即放行路径；不动 J1，只做闭爪 + 收起 */
                (void)Chassis_Path_NotifyTaskDone((uint8_t)'A');
                s_full_t_ms = HAL_GetTick();
                s_full_state = 4U;
            }
            else if (s_full_flow != 0U)
            {
                /* 演示流程的 B/C/D 任务点：当拍放行，与正式模式一致 */
                (void)Chassis_Path_NotifyTaskDone(ev.point_id);
            }
            else
            {
                if (s_key1_scan != 0U && ev.point_id == (uint8_t)'A')
                {
                    /* 键 1：到 A、展开完成后发一次扫码指令 */
                    s_key1_scan = 0U;
                    (void)Vision_Link_RequestScanCode();
                }

                s_task_point = ev.point_id;
                s_task_pending = 1U;
                s_task_done_at = HAL_GetTick() + APP_DEBUG_TASK_MS;
            }
        }
        else if (ev.type == PATH_EV_DONE)
        {
            if (s_chain_active != 0U)
            {
                s_chain_index++;

                if (s_chain_index < APP_DEBUG_PATH_TABLE_COUNT)
                {
                    s_chain_pending = 1U;
                    s_chain_next_ms = HAL_GetTick() + APP_DEBUG_CHAIN_DWELL_MS;
                }
                else
                {
                    s_chain_active = 0U;
                }
            }

            /* 键 3：到 A 后 J1 右转 90°（q1 正 = 左转，故取负） */
            if (s_a_arm_flow != 0U && s_a_arm_state == 0U)
            {
                (void)Arm_Debug_StepJog(-90.0f);
                s_a_j1_start_ms = HAL_GetTick();
                s_a_arm_state = 1U;
            }

            /* 键 9：路径表 DONE 推进演示状态机 */
            if (s_full_flow != 0U)
            {
                if (s_full_state <= 1U)        /* 表 0 DONE（A 动作已由 ARRIVE 启动，忽略） */
                {
                    /* 不动作 */
                }
                else if (s_full_state == 6U)   /* 表 1 到 B */
                {
                    s_full_t_ms = HAL_GetTick();
                    s_full_state = 7U;
                }
                else if (s_full_state == 19U)  /* 表 2 到 C */
                {
                    s_full_t_ms = HAL_GetTick();
                    s_full_state = 20U;
                }
                else if (s_full_state == 29U)  /* 表 3 到 D */
                {
                    s_full_t_ms = HAL_GetTick();
                    s_full_state = 30U;
                }
                else if (s_full_state == 38U)  /* 表 4 到终点，完成 */
                {
                    s_full_flow = 0U;
                    s_full_state = 39U;
                }
            }
        }
    }

    if (s_task_pending != 0U
        && (int32_t)(HAL_GetTick() - s_task_done_at) >= 0)
    {
        s_task_pending = 0U;
        Chassis_Path_NotifyTaskDone(s_task_point);
    }
}

/* 单页调试模式：不做页面切换。 */

void App_Debug_Update(void)
{
    /* S14 键 9：演示模式 A→B→C→D 全流程（动作逐个完成） */
    if (s_full_flow != 0U)
    {
        uint32_t now = HAL_GetTick();

        /* 演示流程期间逐轴直接请求（与 S20 相同路径），保证缓存有效 */
        if ((now - s_full_fb_ms) > 100U)
        {
            s_full_fb_ms = now;
            Arm_Debug_ServoReadTarget(s_demo_read_id);
            s_demo_read_id = (s_demo_read_id >= 3U) ? 1U
                                                    : (uint8_t)(s_demo_read_id + 1U);
        }

        switch (s_full_state)
        {
        case 0U:  /* 应急模式不再使用（键 9 直接从 state 1 开始） */
            break;

        case 1U:  /* 等表 0 DONE（摆臂完成后） */
            break;

        case 4U:  /* A：不动 J1，启动闭爪 + 收起，同时前往 B（不等待回收） */
            (void)Arm_Close();
            (void)Arm_MovePwm(s_pose_stow, DEMO_ARM_MS);
            s_full_t_ms = now;
            s_full_state = 5U;
            break;

        case 5U:  /* A：前往 B */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Chassis_Path_SelectById(1U);
                (void)Chassis_Path_Start();
                s_full_state = 6U;
            }
            break;

        case 6U:  /* 等表 1 DONE */
            break;

        /* ---------- B ---------- */
        case 7U:  /* 张开（并行）+ 通用展开 */
            (void)Arm_Debug_Gripper(0U);
            (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
            s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
            s_full_state = 8U;
            break;

        case 8U:  /* B：步进左转 90°（左展开方向） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 9U;
            }
            break;

        case 9U:  /* B 演示取球 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_b_demo, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS;
                s_full_state = 10U;
            }
            break;

        case 10U: /* 夹取 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_GripperGrasp();
                s_full_t_ms = now + DEMO_GRIP_MS;
                s_full_state = 11U;
            }
            break;

        case 11U: /* 回展开 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 12U;
            }
            break;

        case 12U: /* 步进右转 180°（+90 → −90，转到桶侧） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(-180.0f);
                s_full_t_ms = now + DEMO_J1_180_MS + DEMO_GAP_MS;
                s_full_state = 13U;
            }
            break;

        case 13U: /* B 演示放桶 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_b_bucket, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS;
                s_full_state = 14U;
            }
            break;

        case 14U: /* 张开 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_Gripper(0U);
                s_full_t_ms = now + DEMO_GRIP_MS;
                s_full_state = 15U;
            }
            break;

        case 15U: /* 回展开（右） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 16U;
            }
            break;

        case 16U: /* 步进回正（−90 → 0） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 17U;
            }
            break;

        case 17U: /* 收起 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_stow, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 18U;
            }
            break;

        case 18U: /* 前往 C */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Chassis_Path_SelectById(2U);
                (void)Chassis_Path_Start();
                s_full_state = 19U;
            }
            break;

        case 19U: /* 等表 2 DONE */
            break;

        /* ---------- C ---------- */
        case 20U: /* 通用展开 */
            (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
            s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
            s_full_state = 21U;
            break;

        case 21U: /* 步进左转 90°（左展开方向） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 22U;
            }
            break;

        case 22U: /* C 预存 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_c, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS;
                s_full_state = 23U;
            }
            break;

        case 23U: /* 激光开 2 s */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_Laser(1U);
                s_full_t_ms = now + DEMO_LASER_MS;
                s_full_state = 24U;
            }
            break;

        case 24U: /* 激光关 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_Laser(0U);
                s_full_t_ms = now;
                s_full_state = 25U;
            }
            break;

        case 25U: /* 回展开 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 26U;
            }
            break;

        case 26U: /* 步进回正（+90 → 0） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(-90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 27U;
            }
            break;

        case 27U: /* 收起 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_stow, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 28U;
            }
            break;

        case 28U: /* 前往 D */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Chassis_Path_SelectById(3U);
                (void)Chassis_Path_Start();
                s_full_state = 29U;
            }
            break;

        case 29U: /* 等表 3 DONE */
            break;

        /* ---------- D ---------- */
        case 30U: /* 张开（并行）+ 通用展开 */
            (void)Arm_Debug_Gripper(0U);
            (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
            s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
            s_full_state = 31U;
            break;

        case 31U: /* 步进左转 90°（左展开方向） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 32U;
            }
            break;

        case 32U: /* D 演示 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_d, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS;
                s_full_state = 33U;
            }
            break;

        case 33U: /* 夹取 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_GripperGrasp();
                s_full_t_ms = now + DEMO_GRIP_MS;
                s_full_state = 34U;
            }
            break;

        case 34U: /* 回展开 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_common, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 35U;
            }
            break;

        case 35U: /* 步进回正（+90 → 0） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_StepJog(-90.0f);
                s_full_t_ms = now + DEMO_J1_90_MS + DEMO_GAP_MS;
                s_full_state = 36U;
            }
            break;

        case 36U: /* 收起（夹爪保持夹取） */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Arm_Debug_MovePwm(s_pose_stow, DEMO_ARM_MS);
                s_full_t_ms = now + DEMO_ARM_MS + DEMO_GAP_MS;
                s_full_state = 37U;
            }
            break;

        case 37U: /* 前往终点 */
            if ((int32_t)(now - s_full_t_ms) >= 0)
            {
                (void)Chassis_Path_SelectById(4U);
                (void)Chassis_Path_Start();
                s_full_state = 38U;
            }
            break;

        case 38U: /* 等表 4 DONE，结束 */
            break;

        default:
            break;
        }
    }

    /* S14 键 3：夹爪张开后再摆臂。回读未就绪时 MoveJoint 返回 BUSY，
       每 50 ms 重试，直到启动成功或出现不可恢复错误。 */
    if (s_a_arm_flow != 0U && s_a_arm_state == 3U
        && (int32_t)(HAL_GetTick() - s_a_gripper_ms) >= 0)
    {
        Arm_Status_e rc = Arm_Debug_MoveJoint(s_a_scan_q);

        if (rc == ARM_OK)
        {
            s_a_arm_state = 0U;
        }
        else if (rc == ARM_BUSY)
        {
            s_a_gripper_ms = HAL_GetTick() + 50U;
        }
        else
        {
            /* LIMIT / UNREACHABLE / COMM：放弃摆臂，继续到 A 后转向 */
            s_a_arm_state = 0U;
        }
    }

    /* S14 键 3：J1 转向完成后发扫码指令 0x10 */
    if (s_a_arm_flow != 0U && s_a_arm_state == 1U
        && (HAL_GetTick() - s_a_j1_start_ms) > APP_A_J1_MOVE_MS)
    {
        (void)Vision_Link_RequestScanCode();
        s_a_arm_state = 2U;
        s_a_arm_flow = 0U;
    }

    /* S3：轮询零偏标定完成 */
    if (App_Debug_GetStep() == DEBUG_STEP_S3 && s_s3_calib_active != 0U)
    {
        if (Chassis_CalibratePoll() != 0U)
        {
            s_s3_calib_active = 0U;
        }
    }

    /* 按 2 触发一轮回读：ID1 -> ID2 -> ID3，读完即停 */
    if (App_Debug_GetStep() == DEBUG_STEP_S20
        && s_read_active != 0U)
    {
        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - s_read_next_ms) >= 0)
        {
            Arm_Debug_ServoReadTarget(s_read_id);
            s_read_next_ms = now + 60U;
            s_read_id++;

            if (s_read_id > 3U)
            {
                s_read_active = 0U;
                s_read_done = 1U;
            }
        }
    }

    /* 单阶段调试：不做运行时切页。换测试项请改 APP_DEBUG_FIXED_STEP 后重新编译。 */

    /* S18 动作启动：未就绪时每 50 ms 重试，2 秒超时 */
    if (App_Debug_GetStep() == DEBUG_STEP_S18
        && s_pending_action != ARM_PEND_NONE)
    {
        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - s_pending_next) >= 0)
        {
            Arm_Status_e rc = ARM_OK;

            switch (s_pending_action)
            {
            case ARM_PEND_HOME:  rc = Arm_Debug_Home(); break;
            case ARM_PEND_STOW:  rc = Arm_Debug_Stow(); break;
            case ARM_PEND_JOINT: rc = Arm_Debug_MoveTestJoint(); break;
            case ARM_PEND_POSE:  rc = Arm_Debug_MoveTestPose(); break;
            default: break;
            }

            if (rc == ARM_OK)
            {
                s_pending_action = ARM_PEND_NONE;
                s_action_error = ARM_OK;
            }
            else if (rc == ARM_BUSY)
            {
                if ((now - s_pending_since) > 2000U)
                {
                    s_pending_action = ARM_PEND_NONE;
                    s_action_error = ARM_ERR_COMM;
                }
                else
                {
                    s_pending_next = now + 50U;
                }
            }
            else
            {
                s_pending_action = ARM_PEND_NONE;
                s_action_error = rc;
            }
        }
    }

    /* S5 的脉冲目标检查（非阻塞） */
    Chassis_Debug_Update();

    /* 整条路径串行执行：停留结束后启动下一段 */
    if (s_chain_pending != 0U
        && (int32_t)(HAL_GetTick() - s_chain_next_ms) >= 0)
    {
        s_chain_pending = 0U;
        (void)Chassis_Path_SelectById(s_chain_index);
        (void)Chassis_Path_Start();
    }

    if (s_jog_active == 0U)
    {
        return;
    }

    if ((int32_t)(HAL_GetTick() - s_jog_end_ms) >= 0)
    {
        s_jog_active = 0U;
        Chassis_Debug_ReleaseMotor();
        return;
    }

    Chassis_Debug_SetMotor(s_jog_l, s_jog_r);
}

void App_Debug_HandleKey(uint8_t key)
{
    if (key == 0U)
    {
        return;
    }

#if (APP_STAGE == APP_STAGE_MISSION)
    /* 正式模式：9 = 启动，8 = 只到 A，7 = 模拟扫码失败跑演示，6 = 中止 */
    if (key == (uint8_t)'9')
    {
        (void)Mission_Start();
    }
    else if (key == (uint8_t)'8')
    {
        (void)Mission_StartToA();
    }
    else if (key == (uint8_t)'7')
    {
        /* 从起点直接跑演示流程，用于模拟 A 点扫码失败；仅空闲时可启动 */
        if (Mission_GetState() == MISSION_STATE_IDLE)
        {
            App_Debug_StartDemo();
        }
    }
    else if (key == (uint8_t)'6')
    {
        Mission_RequestAbort();
    }
    s_last_key = key;
    return;
#endif

    /* 硬约束：此处禁止任何阻塞调用 */
    switch (s_debug_step)
    {
    case DEBUG_STEP_S15:
        /* J1 步进方向测试：1/2 = +90°/−90°，3 = 停止，7/8 = 使能/失能，9 = 回读 */
        switch (key)
        {
        case (uint8_t)'1': (void)Arm_Debug_StepJog(90.0f); break;
        case (uint8_t)'2': (void)Arm_Debug_StepJog(-90.0f); break;
        case (uint8_t)'3': (void)Arm_Debug_StepStop(); break;
        case (uint8_t)'7': (void)Arm_Debug_StepEnable(1U); break;
        case (uint8_t)'8': (void)Arm_Debug_StepEnable(0U); break;
        case (uint8_t)'9': Arm_Debug_StepRead(); break;
        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S16:
        /* 1/2/3 = 选择 J2/J3/J4，6 = 扭矩切换，7/8 = -/+5°，9 = 回读 */
        switch (key)
        {
        case (uint8_t)'1': s_arm_axis = 0U; break;
        case (uint8_t)'2': s_arm_axis = 1U; break;
        case (uint8_t)'3': s_arm_axis = 2U; break;
        case (uint8_t)'6':
            s_tq_on ^= 1U;
            Arm_Debug_ServoTorqueBroadcast(s_tq_on);
            break;
        case (uint8_t)'7':
            (void)Arm_Debug_ServoJog((uint8_t)(ARM_AXIS_J2 + s_arm_axis), -5.0f);
            break;
        case (uint8_t)'8':
            (void)Arm_Debug_ServoJog((uint8_t)(ARM_AXIS_J2 + s_arm_axis), 5.0f);
            break;
        case (uint8_t)'9': Arm_Debug_ServoRead(); break;
        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S17:
        /* 1/2 = 夹爪张开/闭合，3/7 = 激光开/关，9 = 回读 */
        switch (key)
        {
        case (uint8_t)'1': (void)Arm_Debug_Gripper(0U); break;
        case (uint8_t)'2': (void)Arm_Debug_Gripper(1U); break;
        case (uint8_t)'3': (void)Arm_Debug_Laser(1U); break;
        case (uint8_t)'7': (void)Arm_Debug_Laser(0U); break;
        case (uint8_t)'9':
            Arm_Debug_StepRead();
            Arm_Debug_ServoRead();
            break;
        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S18:
        /* 1 = 测试位姿 (0,0,400)，2 = Stow，3 = 停止，7 = 测试关节，9 = 回读；
           动作先登记，由 App_Debug_Update 处理反馈就绪后重试启动 */
        switch (key)
        {
        case (uint8_t)'1':
            if (Arm_IsBusy() == 0U && s_pending_action == ARM_PEND_NONE)
            {
                s_pending_action = ARM_PEND_POSE;
                s_pending_since = HAL_GetTick();
                s_pending_next = 0U;
                s_action_error = ARM_OK;
            }
            break;

        case (uint8_t)'2':
            if (Arm_IsBusy() == 0U && s_pending_action == ARM_PEND_NONE)
            {
                s_pending_action = ARM_PEND_STOW;
                s_pending_since = HAL_GetTick();
                s_pending_next = 0U;
                s_action_error = ARM_OK;
            }
            break;

        case (uint8_t)'3':
            Arm_Stop();
            s_pending_action = ARM_PEND_NONE;
            s_action_error = ARM_OK;
            break;

        case (uint8_t)'6':
            /* 竖直直臂：q2=90、q3=0、q4=0，用于验证 J2/J3 标定 */
            if (Arm_IsBusy() == 0U && s_pending_action == ARM_PEND_NONE)
            {
                s_pending_action = ARM_PEND_JOINT;
                s_pending_since = HAL_GetTick();
                s_pending_next = 0U;
                s_action_error = ARM_OK;
            }
            break;

        case (uint8_t)'7':
            /* 大臂 J2 点动 −5°（朝水平前伸方向）：先按 9 取 PRAD 基线 */
            s_action_error = Arm_Debug_ServoJog(ARM_AXIS_J2, -5.0f);
            break;

        case (uint8_t)'8':
            /* 大臂 J2 点动 +5°（朝竖直方向） */
            s_action_error = Arm_Debug_ServoJog(ARM_AXIS_J2, 5.0f);
            break;

        case (uint8_t)'9':
            Arm_Debug_StepRead();
            Arm_Debug_ServoRead();
            break;

        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S19:
        /* 1/2/3 = 目标 ID，7 = 改 ID（旧 ID 0），8 = 读 ID，9 = 读目标位置 */
        switch (key)
        {
        case (uint8_t)'1': s_id_target = 1U; break;
        case (uint8_t)'2': s_id_target = 2U; break;
        case (uint8_t)'3': s_id_target = 3U; break;
        case (uint8_t)'7':
            (void)Arm_Debug_ServoSetId(0U, s_id_target);
            break;
        case (uint8_t)'8': Arm_Debug_ServoRequestId(0U); break;
        case (uint8_t)'9': Arm_Debug_ServoReadTarget(s_id_target); break;
        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S20:
        /* 1=扭矩切换，2=张爪(45°)，3=闭爪(135°)，7/8=设 ID1/ID3 启动位，9=回读三轴 */
        switch (key)
        {
        case (uint8_t)'1':
            s_show_scan = 0U;
            s_tq_on ^= 1U;
            Arm_Debug_ServoTorqueBroadcast(s_tq_on);
            break;

        case (uint8_t)'2':
            /* 张爪：45° → 833 us */
            s_action_error = Arm_Debug_Gripper(0U);
            break;

        case (uint8_t)'3':
            /* 闭爪：135° → 1500 us */
            s_action_error = Arm_Debug_Gripper(1U);
            break;

        case (uint8_t)'5':
            /* 修改 P1（ID1 / J2）初始值 */
            s_show_scan = 0U;
            Arm_Debug_ServoSetStart(1U);
            break;

        case (uint8_t)'7':
            /* 激光开/关切换 */
            s_show_scan = 0U;
            s_laser_on ^= 1U;
            (void)Arm_Debug_Laser(s_laser_on);
            break;

        case (uint8_t)'8':
            s_show_scan = 0U;
            Arm_Debug_ServoSetStart(3U);            /* P3 = ID3 */
            break;

        case (uint8_t)'9':
            s_show_scan = 0U;
            Arm_Debug_ServoClearCache();
            Arm_Debug_StepRead();          /* 同时请求 J1 步进位置 */
            s_read_active = 1U;
            s_read_done = 0U;
            s_read_id = 1U;
            s_read_next_ms = 0U;
            break;

        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S21:
        /* J3 换装（ID2）：1=扫描 0~10，2=旧 ID0→2，3=旧 ID1→2，
           6=扭矩切换，7=回读 ID2，8=J3 +5°，9=写 ID2 启动位置 */
        switch (key)
        {
        case (uint8_t)'1':
            Arm_Debug_ServoClearCache();
            Arm_Debug_ServoScanStart();
            break;

        case (uint8_t)'2':
            (void)Arm_Debug_ServoSetId(0U, 2U);
            break;

        case (uint8_t)'3':
            /* 写 ID2 启动位置 */
            s_show_scan = 0U;
            Arm_Debug_ServoSetStart(2U);
            break;

        case (uint8_t)'6':
            s_tq_on ^= 1U;
            Arm_Debug_ServoTorqueBroadcast(s_tq_on);
            break;

        case (uint8_t)'7':
            Arm_Debug_ServoClearCache();
            Arm_Debug_ServoReadTarget(2U);
            break;

        case (uint8_t)'8':
            /* J3 正向点动 +5°（收起方向） */
            s_action_error = Arm_Debug_ServoJog(ARM_AXIS_J3, 5.0f);
            break;

        case (uint8_t)'9':
            /* J3 反向点动 −5°（伸直方向） */
            s_action_error = Arm_Debug_ServoJog(ARM_AXIS_J3, -5.0f);
            break;

        default: break;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S14:
    {
        /* 按键映射（4 硬件不可用）：
           1 = 起点→A（表 0），2 = A→B（表 1），
           3 = 起点→A + 机械臂扫码流程，5 = 纯底盘整链，
           6 = 中止，7 = C→D（表 3），8 = D→终点（表 4），9 = 演示整链 */
        uint16_t table_id = 0xFFFFU;

        switch (key)
        {
        case (uint8_t)'1':
            /* 跑到 A 并展开：张爪 + 腕部平伸，随后发车表 0 */
            (void)Arm_Debug_Gripper(0U);
            (void)Arm_SetWristPwm(1559U, DEMO_ARM_MS);
            s_key1_scan = 1U;
            table_id = 0U;
            break;

        case (uint8_t)'2': table_id = 1U; break;

        case (uint8_t)'3':
        {
            /* 先张爪 45°，800 ms 后再摆到 A 扫码姿态（J2 1976/J3 2023/J4 1829），
               同时发车；到 A 后只动 J1 步进右转 90°，再发扫码 0x10 */
            (void)Arm_Debug_Gripper(0U);
            s_a_gripper_ms = HAL_GetTick() + APP_A_GRIPPER_MS;
            s_a_arm_flow = 1U;
            s_a_arm_state = 3U;
            table_id = 0U;
            break;
        }

        case (uint8_t)'5':
            /* 纯底盘整链：0→1→2→3→4，每段完成停 2 s，不做机械臂动作 */
            s_chain_active = 1U;
            s_chain_index = 0U;
            s_chain_pending = 0U;
            (void)Chassis_Path_SelectById(0U);
            (void)Chassis_Path_Start();
            break;

        case (uint8_t)'7': table_id = 3U; break;

        case (uint8_t)'8':
            /* D→终点（表 4），与正式模式路径一致 */
            table_id = 4U;
            break;

        case (uint8_t)'9':
            App_Debug_StartDemo();
            break;

        case (uint8_t)'6':
            /* 中止：清链式推进，停路径与电机 */
            s_chain_active = 0U;
            s_chain_pending = 0U;
            Chassis_Path_Abort();
            Chassis_Control_Stop(1U);
            break;

        default:
            break;
        }

        if (table_id != 0xFFFFU)
        {
            (void)Chassis_Path_SelectById(table_id);
            (void)Chassis_Path_Start();
        }

        s_last_key = key;
        break;
    }

    case DEBUG_STEP_S12:
        /* 固定工况：R200，±90°。1/3 = 左转 90°，2 = 右转 90°，7 = 停止 */
        if (key == (uint8_t)'1')
        {
            s_dbg_radius = 200.0f; s_dbg_angle = 90.0f;
            Chassis_Debug_RunArc(200.0f, 90.0f, 200.0f);
        }
        else if (key == (uint8_t)'2')
        {
            s_dbg_radius = -200.0f; s_dbg_angle = -90.0f;
            Chassis_Debug_RunArc(-200.0f, -90.0f, 200.0f);
        }
        else if (key == (uint8_t)'3')
        {
            s_dbg_radius = 200.0f; s_dbg_angle = 90.0f;
            Chassis_Debug_RunArc(200.0f, 90.0f, 200.0f);
        }
        else if (key == (uint8_t)'7')
        {
            Chassis_Debug_StopStraight();
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S11:
        /* 1 = 左转90°，2 = 右转90°，3 = 左转180°，7 = 停止 */
        if (key == (uint8_t)'1')
        {
            s_dbg_angle = 90.0f;
            Chassis_Debug_RunSpin(90.0f);
        }
        else if (key == (uint8_t)'2')
        {
            s_dbg_angle = -90.0f;
            Chassis_Debug_RunSpin(-90.0f);
        }
        else if (key == (uint8_t)'3')
        {
            s_dbg_angle = 180.0f;
            Chassis_Debug_RunSpin(180.0f);
        }
        else if (key == (uint8_t)'7')
        {
            Chassis_Debug_StopStraight();
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S10:
        /* 1/2 = 直行 1000mm @ 300/200 mm/s；7 = 急停 */
        if (key == (uint8_t)'1')
        {
            s_dbg_dist = 1000.0f; s_dbg_speed = 300.0f;
            Chassis_Debug_RunStraight(1000.0f, 300.0f);
        }
        else if (key == (uint8_t)'2')
        {
            s_dbg_dist = 1000.0f; s_dbg_speed = 200.0f;
            Chassis_Debug_RunStraight(1000.0f, 200.0f);
        }
        else if (key == (uint8_t)'7')
        {
            Chassis_Debug_StopStraight();
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S9:
        /* 1/2/3 = 双轮同速 200/400/600 mm/s；7/8 = 单轮 400；9 = 停止 */
        if (key == (uint8_t)'1')
        {
            s_dbg_tl = 200.0f; s_dbg_tr = 200.0f;
            Chassis_Debug_RunSpeed(200.0f, 200.0f);
        }
        else if (key == (uint8_t)'2')
        {
            s_dbg_tl = 400.0f; s_dbg_tr = 400.0f;
            Chassis_Debug_RunSpeed(400.0f, 400.0f);
        }
        else if (key == (uint8_t)'3')
        {
            s_dbg_tl = 600.0f; s_dbg_tr = 600.0f;
            Chassis_Debug_RunSpeed(600.0f, 600.0f);
        }
        else if (key == (uint8_t)'7')
        {
            s_dbg_tl = 400.0f; s_dbg_tr = 0.0f;
            Chassis_Debug_RunSpeed(400.0f, 0.0f);
        }
        else if (key == (uint8_t)'8')
        {
            s_dbg_tl = 0.0f; s_dbg_tr = 400.0f;
            Chassis_Debug_RunSpeed(0.0f, 400.0f);
        }
        else if (key == (uint8_t)'9')
        {
            s_dbg_tl = 0.0f; s_dbg_tr = 0.0f;
            Chassis_Debug_StopSpeed();
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S5:
        /* 1/2/3 = 前进指定脉冲数后制动；7 = 立即停止
           实测约 0.003 mm/脉冲，故档位取 10 万级（约 300/600/900 mm） */
        if (key == (uint8_t)'1')
        {
            Chassis_Debug_RunPulses(100000, APP_DEBUG_PULSE_PWM);
        }
        else if (key == (uint8_t)'2')
        {
            Chassis_Debug_RunPulses(200000, APP_DEBUG_PULSE_PWM);
        }
        else if (key == (uint8_t)'3')
        {
            Chassis_Debug_RunPulses(300000, APP_DEBUG_PULSE_PWM);
        }
        else if (key == (uint8_t)'7')
        {
            Chassis_Debug_StopPulses();
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S4:
        debug_start_jog(key);
        s_last_key = key;
        break;

    case DEBUG_STEP_S3:
        /* 1 = 航向清零；2 = 重新做零偏标定（标定期间保持静止） */
        if (key == (uint8_t)'1')
        {
            Chassis_ResetHeading();
        }
        else if (key == (uint8_t)'2')
        {
            Chassis_CalibrateStart();
            s_s3_calib_active = 1U;
        }
        s_last_key = key;
        break;

    case DEBUG_STEP_S1:
    default:
        s_last_key = key;
        break;
    }
}

static char *dbg_stpcpy(char *dst, const char *src)
{
    while (*src != '\0')
    {
        *dst++ = *src++;
    }
    *dst = '\0';
    return dst;
}

void App_Dbg_FmtInt(char *buf, int32_t v)
{
    char tmp[12];
    uint8_t i = 0U;
    uint32_t u;
    char *p = buf;

    if (v < 0)
    {
        *p++ = '-';
        u = (uint32_t)(-(v + 1)) + 1U;
    }
    else
    {
        u = (uint32_t)v;
    }

    if (u == 0U)
    {
        tmp[i++] = '0';
    }
    else
    {
        while (u > 0U)
        {
            tmp[i++] = (char)('0' + (u % 10U));
            u /= 10U;
        }
    }

    while (i > 0U)
    {
        *p++ = tmp[--i];
    }
    *p = '\0';
}

void App_Dbg_FmtFloat(char *buf, float v, uint8_t decimals)
{
    int32_t scale = 1;
    int32_t n;
    int32_t frac;
    int32_t div;
    uint8_t i;
    char *p;

    if (decimals > 4U)
    {
        decimals = 4U;
    }

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    n = (int32_t)(v * (float)scale + ((v >= 0.0f) ? 0.5f : -0.5f));

    App_Dbg_FmtInt(buf, n / scale);

    p = buf;
    while (*p != '\0')
    {
        p++;
    }

    if (decimals > 0U)
    {
        frac = n % scale;
        if (frac < 0)
        {
            frac = -frac;
        }

        *p++ = '.';

        div = 1;
        for (i = 1U; i < decimals; i++)
        {
            div *= 10;
        }

        for (i = 0U; i < decimals; i++)
        {
            *p++ = (char)('0' + (frac / div) % 10);
            if (div > 1)
            {
                div /= 10;
            }
        }

        *p = '\0';
    }
}

static void arm_dbg_put_float(char **p, float v, uint8_t dec)
{
    char num[16];

    App_Dbg_FmtFloat(num, v, dec);
    *p = dbg_stpcpy(*p, num);
}

static void arm_dbg_put_int(char **p, int32_t v)
{
    char num[16];

    App_Dbg_FmtInt(num, v);
    *p = dbg_stpcpy(*p, num);
}

static const char *arm_stage_title(App_Debug_Step_e step)
{
    switch (step)
    {
    case DEBUG_STEP_S15: return "J1 STEP";
    case DEBUG_STEP_S16: return "SERVO JOG";
    case DEBUG_STEP_S17: return "GRIP/LASER";
    case DEBUG_STEP_S18: return "PTP ACTION";
    case DEBUG_STEP_S19: return "SET ID";
    case DEBUG_STEP_S20: return "SERVO CAL";
    case DEBUG_STEP_S21: return "J3 REPLACE";
    default:             return debug_step_name(step);
    }
}

static const char *id_result_str(uint8_t result)
{
    switch (result)
    {
    case 1U: return "OK";
    case 2U: return "FAIL";
    default: return "WAIT";
    }
}

static const char *arm_action_name(uint8_t action)
{
    switch (action)
    {
    case 1U: return "HOME";
    case 2U: return "STOW";
    case 3U: return "JOINT";
    case 4U: return "POSE";
    case 5U: return "P&P";
    case 6U: return "AIM";
    case 7U: return "HOLD";
    case 8U: return "REL";
    default: return "-";
    }
}

static void arm_scan_refresh(void)
{
    char line[DEBUG_LINE_MAX];
    char *p;
    uint8_t i;
    uint8_t count = Arm_Debug_ServoScanCount();

    p = line;
    p = dbg_stpcpy(p, "RANGE 0-10");
    *p = '\0';
    Oled_DrawStrLine(1U, line);

    p = line;
    p = dbg_stpcpy(p, (Arm_Debug_ServoScanIsActive() != 0U)
                        ? "SCAN RUNNING" : "SCAN DONE");
    *p = '\0';
    Oled_DrawStrLine(2U, line);

    p = line;
    p = dbg_stpcpy(p, "FOUND ");
    arm_dbg_put_int(&p, (int32_t)count);
    *p = '\0';
    Oled_DrawStrLine(3U, line);

    p = line;
    p = dbg_stpcpy(p, "IDS ");
    for (i = 0U; i < count && i < 6U; i++)
    {
        uint8_t id = Arm_Debug_ServoScanId(i);
        if (id == 0xFFU)
        {
            break;
        }
        arm_dbg_put_int(&p, (int32_t)id);
        *p++ = ' ';
    }
    *p = '\0';
    Oled_DrawStrLine(4U, line);

    p = line;
    for (i = 6U; i < count; i++)
    {
        uint8_t id = Arm_Debug_ServoScanId(i);
        if (id == 0xFFU)
        {
            break;
        }
        arm_dbg_put_int(&p, (int32_t)id);
        *p++ = ' ';
    }
    *p = '\0';
    Oled_DrawStrLine(5U, line);

    p = line;
    p = dbg_stpcpy(p, "2 ID0>2 3 ID2>1");
    *p = '\0';
    Oled_DrawStrLine(6U, line);

    p = line;
    p = dbg_stpcpy(p, "1 RESCAN 7TQ 8/9ST");
    *p = '\0';
    Oled_DrawStrLine(7U, line);
}

static void arm_dbg_refresh(App_Debug_Step_e step, uint8_t last_key)
{
    Arm_Debug_View_t v;
    char line[DEBUG_LINE_MAX];
    char *p;
    uint16_t pwm1;
    uint16_t pwm2;
    uint16_t pwm3;

    Arm_Debug_GetView(&v);
    Oled_Clear();

    p = line;
    p = dbg_stpcpy(p, "ARM ");
    p = dbg_stpcpy(p, (step == DEBUG_STEP_S20 && s_show_scan != 0U)
                        ? "SERVO SCAN" : arm_stage_title(step));
    p = dbg_stpcpy(p, " K:");
    *p++ = (last_key != 0U) ? (char)last_key : '-';
    *p = '\0';
    Oled_DrawStrLine(0U, line);

    if (step == DEBUG_STEP_S15)
    {
        p = line;
        p = dbg_stpcpy(p, "J1 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J1], 1U);
        p = dbg_stpcpy(p, " DEG");
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        p = dbg_stpcpy(p, v.step_online ? "STEP ON E" : "STEP OFF E");
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J1]);
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "K");
        arm_dbg_put_int(&p, (int32_t)v.step_err_kind);
        p = dbg_stpcpy(p, " A");
        arm_dbg_put_int(&p, (int32_t)v.step_last_ack);
        p = dbg_stpcpy(p, " FW");
        arm_dbg_put_int(&p, (int32_t)v.step_fw);
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "HW");
        arm_dbg_put_int(&p, (int32_t)v.step_hw);
        *p++ = ' ';
        p = dbg_stpcpy(p, v.gripper_closed ? "G C " : "G O ");
        p = dbg_stpcpy(p, v.laser_on ? "L ON" : "L OFF");
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "J2 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J2], 1U);
        p = dbg_stpcpy(p, " J3 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J3], 1U);
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "J4 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J4], 1U);
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        p = dbg_stpcpy(p, "1+90 2-90 3STOP");
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }
    else if (step == DEBUG_STEP_S16)
    {
        p = line;
        p = dbg_stpcpy(p, "SEL J");
        arm_dbg_put_int(&p, (int32_t)(s_arm_axis + 2U));
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        p = dbg_stpcpy(p, "J2 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J2], 1U);
        p = dbg_stpcpy(p, " J3 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J3], 1U);
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "J4 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J4], 1U);
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "PWM ");
        arm_dbg_put_int(&p, (int32_t)v.servo_pwm[s_arm_axis]);
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "ON ");
        *p++ = v.axis_online[ARM_AXIS_J2] ? '2' : '-';
        *p++ = v.axis_online[ARM_AXIS_J3] ? '3' : '-';
        *p++ = v.axis_online[ARM_AXIS_J4] ? '4' : '-';
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "ERR ");
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J2]);
        *p++ = '/';
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J3]);
        *p++ = '/';
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J4]);
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        p = dbg_stpcpy(p, "1/2/3SEL 6TQ 7-8+ 9RD");
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }
    else if (step == DEBUG_STEP_S17)
    {
        p = line;
        p = dbg_stpcpy(p, v.gripper_closed ? "GRIP CLOSE" : "GRIP OPEN");
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        if (v.laser_available == 0U)
        {
            p = dbg_stpcpy(p, "LASER N/A");
        }
        else
        {
            p = dbg_stpcpy(p, v.laser_on ? "LASER ON" : "LASER OFF");
        }
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "ON ");
        *p++ = v.axis_online[ARM_AXIS_J1] ? '1' : '-';
        *p++ = v.axis_online[ARM_AXIS_J2] ? '2' : '-';
        *p++ = v.axis_online[ARM_AXIS_J3] ? '3' : '-';
        *p++ = v.axis_online[ARM_AXIS_J4] ? '4' : '-';
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "ERR ");
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J1]);
        *p++ = '/';
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J2]);
        *p++ = '/';
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J3]);
        *p++ = '/';
        arm_dbg_put_int(&p, (int32_t)v.err[ARM_AXIS_J4]);
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "1OP 2CL 3LON 7LOFF");
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "9 READ ALL");
        *p = '\0';
        Oled_DrawStrLine(6U, line);
    }
    else if (step == DEBUG_STEP_S18)
    {
        uint8_t act_name = (s_pending_action != ARM_PEND_NONE)
                         ? (uint8_t)s_pending_action : v.action;
        Arm_Status_e err = (s_action_error != ARM_OK)
                         ? s_action_error : v.status;

        p = line;
        p = dbg_stpcpy(p, "ACT ");
        p = dbg_stpcpy(p, arm_action_name(act_name));
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        p = dbg_stpcpy(p, (s_pending_action != ARM_PEND_NONE)
                            ? "WAIT" : ((v.motion_busy != 0U) ? "BUSY" : "IDLE"));
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "ERR ");
        arm_dbg_put_int(&p, (int32_t)err);
        p = dbg_stpcpy(p, " ON ");
        *p++ = v.axis_online[ARM_AXIS_J1] ? '1' : '-';
        *p++ = v.axis_online[ARM_AXIS_J2] ? '2' : '-';
        *p++ = v.axis_online[ARM_AXIS_J3] ? '3' : '-';
        *p++ = v.axis_online[ARM_AXIS_J4] ? '4' : '-';
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "J1 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J1], 1U);
        p = dbg_stpcpy(p, " J2 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J2], 1U);
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "J3 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J3], 1U);
        p = dbg_stpcpy(p, " J4 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J4], 1U);
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "1POSE 2STOW 3STOP");
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        p = dbg_stpcpy(p, "6JNT 7J2- 8J2+ 9RD");
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }
    else if (step == DEBUG_STEP_S19)
    {
        p = line;
        p = dbg_stpcpy(p, "TARGET ID ");
        arm_dbg_put_int(&p, (int32_t)s_id_target);
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        p = dbg_stpcpy(p, "LAST ID ");
        if (v.servo_id_valid != 0U)
        {
            arm_dbg_put_int(&p, (int32_t)v.servo_last_id);
        }
        else
        {
            p = dbg_stpcpy(p, "--");
        }
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "CHK ");
        p = dbg_stpcpy(p, id_result_str(v.servo_id_result));
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "7SET 8RDID 9RDPOS");
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "ONE SERVO ONLY");
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "POWER CYCLE SET");
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        p = dbg_stpcpy(p, "LONG3 NEXTPAGE");
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }
    else if (step == DEBUG_STEP_S21)
    {
        p = line;
        p = dbg_stpcpy(p, "ID2 PWM ");
        if (Arm_Debug_ServoGetPosition(2U, &pwm2) == 0)
        {
            arm_dbg_put_int(&p, (int32_t)pwm2);
        }
        else
        {
            p = dbg_stpcpy(p, "--");
        }
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        if (Arm_Debug_ServoScanIsActive() != 0U)
        {
            p = dbg_stpcpy(p, "SCANNING");
        }
        else if (Arm_Debug_ServoScanIsDone() != 0U)
        {
            uint8_t i;
            uint8_t n = Arm_Debug_ServoScanCount();

            p = dbg_stpcpy(p, "IDS ");
            if (n == 0U)
            {
                p = dbg_stpcpy(p, "NONE");
            }
            else
            {
                for (i = 0U; i < n && i < 6U; i++)
                {
                    arm_dbg_put_int(&p, (int32_t)Arm_Debug_ServoScanId(i));
                    *p++ = ' ';
                }
            }
        }
        else
        {
            p = dbg_stpcpy(p, "PRESS 1 TO SCAN");
        }
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, (s_tq_on != 0U) ? "TQ ON  ERR " : "TQ OFF ERR ");
        arm_dbg_put_int(&p, (int32_t)s_action_error);
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, "1SCAN 2ID0>2 3START");
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "6TQ 7RD 8J3+ 9J3-");
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "ONE SERVO ONLY");
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        p = dbg_stpcpy(p, "ID CHANGE: REPOWER");
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }
    else if (step == DEBUG_STEP_S20 && s_show_scan != 0U)
    {
        arm_scan_refresh();
    }
    else
    {
        p = line;
        p = dbg_stpcpy(p, "ID1 POS ");
        if (Arm_Debug_ServoGetPosition(1U, &pwm1) == 0)
        {
            arm_dbg_put_int(&p, (int32_t)pwm1);
        }
        else
        {
            p = dbg_stpcpy(p, "--");
        }
        *p = '\0';
        Oled_DrawStrLine(1U, line);

        p = line;
        p = dbg_stpcpy(p, "ID2 POS ");
        if (Arm_Debug_ServoGetPosition(2U, &pwm2) == 0)
        {
            arm_dbg_put_int(&p, (int32_t)pwm2);
        }
        else
        {
            p = dbg_stpcpy(p, "--");
        }
        *p = '\0';
        Oled_DrawStrLine(2U, line);

        p = line;
        p = dbg_stpcpy(p, "ID3 POS ");
        if (Arm_Debug_ServoGetPosition(3U, &pwm3) == 0)
        {
            arm_dbg_put_int(&p, (int32_t)pwm3);
        }
        else
        {
            p = dbg_stpcpy(p, "--");
        }
        *p = '\0';
        Oled_DrawStrLine(3U, line);

        p = line;
        p = dbg_stpcpy(p, (s_tq_on != 0U) ? "TQ ON  J1 " : "TQ OFF J1 ");
        arm_dbg_put_float(&p, v.q_deg[ARM_AXIS_J1], 1U);
        *p = '\0';
        Oled_DrawStrLine(4U, line);

        p = line;
        p = dbg_stpcpy(p, "1TQ 2OPEN 3CLOSE");
        *p = '\0';
        Oled_DrawStrLine(5U, line);

        p = line;
        p = dbg_stpcpy(p, "7LASER 8ST3 9RD");
        *p = '\0';
        Oled_DrawStrLine(6U, line);

        p = line;
        if (s_read_active != 0U)
        {
            p = dbg_stpcpy(p, "READING ");
            arm_dbg_put_int(&p, (int32_t)s_read_id);
            p = dbg_stpcpy(p, "/3");
        }
        else if (s_read_done != 0U)
        {
            p = dbg_stpcpy(p, "READ DONE");
        }
        else
        {
            p = dbg_stpcpy(p, "9 READ POS");
        }
        *p = '\0';
        Oled_DrawStrLine(7U, line);
    }

    Oled_Flush();
}

static void chassis_s3_refresh(void)
{
    Chassis_Debug_View_t v;
    char line[DEBUG_LINE_MAX];
    char num[16];
    char *p;
    uint32_t now_ms;
    uint32_t dt_ms;

    Chassis_Debug_GetView(&v);

    now_ms = HAL_GetTick();
    dt_ms = now_ms - s_rate_last_tick;
    if (dt_ms >= 1000U)
    {
        if (s_rate_last_tick != 0U)
        {
            s_frame_rate = (v.gyro_seq - s_rate_last_seq) * 1000U / dt_ms;
        }
        s_rate_last_seq = v.gyro_seq;
        s_rate_last_tick = now_ms;
    }

    Oled_Clear();

    p = line;
    p = dbg_stpcpy(p, "26GX K:");
    *p++ = (App_Debug_GetLastKey() != 0U) ? (char)App_Debug_GetLastKey() : '-';
    p = dbg_stpcpy(p, " S3");
    *p = '\0';
    Oled_DrawStrLine(0U, line);

    p = line;
    p = dbg_stpcpy(p, "YAW ");
    App_Dbg_FmtFloat(num, v.yaw_gyro_deg, 1U);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(1U, line);

    p = line;
    p = dbg_stpcpy(p, "W ");
    App_Dbg_FmtFloat(num, v.wz_dps, 2U);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(2U, line);

    p = line;
    p = dbg_stpcpy(p, "R ");
    App_Dbg_FmtInt(num, (int32_t)s_frame_rate);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(3U, line);

    p = line;
    p = dbg_stpcpy(p, "Y");
    App_Dbg_FmtInt(num, (int32_t)v.gyro_yaw_frames);
    p = dbg_stpcpy(p, num);
    p = dbg_stpcpy(p, " W");
    App_Dbg_FmtInt(num, (int32_t)v.gyro_wz_frames);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(4U, line);

    p = line;
    p = dbg_stpcpy(p, "RX ");
    App_Dbg_FmtInt(num, (int32_t)v.gyro_rx_bytes);
    p = dbg_stpcpy(p, num);
    p = dbg_stpcpy(p, " E");
    App_Dbg_FmtInt(num, (int32_t)v.gyro_err);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(5U, line);

    p = line;
    if (s_s3_calib_active != 0U)
    {
        p = dbg_stpcpy(p, "CAL ");
        App_Dbg_FmtInt(num, (int32_t)v.gyro_calib_cnt);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, "/");
        App_Dbg_FmtInt(num, (int32_t)v.gyro_calib_target);
        p = dbg_stpcpy(p, num);
    }
    else if (v.calib_done != 0U)
    {
        p = dbg_stpcpy(p, "CAL OK");
    }
    else
    {
        p = dbg_stpcpy(p, "CAL --");
    }
    *p = '\0';
    Oled_DrawStrLine(6U, line);

    p = line;
    p = dbg_stpcpy(p, "PWM ");
    App_Dbg_FmtInt(num, (int32_t)v.pwm_l);
    p = dbg_stpcpy(p, num);
    p = dbg_stpcpy(p, "/");
    App_Dbg_FmtInt(num, (int32_t)v.pwm_r);
    p = dbg_stpcpy(p, num);
    p = dbg_stpcpy(p, (v.gyro_online != 0U) ? " G" : " L");
    *p = '\0';
    Oled_DrawStrLine(7U, line);

    Oled_Flush();
}

void App_Debug_Refresh(void)
{
    if (App_Debug_GetStep() == DEBUG_STEP_S3)
    {
        chassis_s3_refresh();
        return;
    }

    if (App_Debug_GetStep() >= DEBUG_STEP_S15)
    {
        arm_dbg_refresh(App_Debug_GetStep(), App_Debug_GetLastKey());
        return;
    }

    Chassis_Debug_View_t v;
    char line[DEBUG_LINE_MAX];
    char num[16];
    char *p;
    uint8_t last_key;
    uint32_t now_ms;
    uint32_t dt_ms;
    float yaw_show;
    int32_t pulse_l;
    int32_t pulse_r;

    Chassis_Debug_GetView(&v);
    Chassis_Debug_GetPulses(&pulse_l, &pulse_r);
    last_key = App_Debug_GetLastKey();

    /* 帧率统计：每秒更新一次，非阻塞 */
    now_ms = HAL_GetTick();
    dt_ms = now_ms - s_rate_last_tick;
    if (dt_ms >= 1000U)
    {
        if (s_rate_last_tick != 0U)
        {
            s_frame_rate = (v.gyro_seq - s_rate_last_seq) * 1000U / dt_ms;
        }
        s_rate_last_seq = v.gyro_seq;
        s_rate_last_tick = now_ms;
    }

    /* S3 用纯陀螺仪航向做标定；其余步骤看融合航向 */
    yaw_show = (App_Debug_GetStep() == DEBUG_STEP_S3)
             ? v.yaw_gyro_deg : v.heading_deg;

    Oled_Clear();

    p = line;
    p = dbg_stpcpy(p, "26GX K:");
    *p++ = (last_key != 0U) ? (char)last_key : '-';
    *p++ = ' ';
    p = dbg_stpcpy(p, debug_step_name(App_Debug_GetStep()));
    p = dbg_stpcpy(p, (s_gyro_calib_state == 1U) ? " GOK"
                   : (s_gyro_calib_state == 2U) ? " GERR" : " GCAL");
    *p = '\0';
    Oled_DrawStrLine(0U, line);

    p = line;
    p = dbg_stpcpy(p, "YAW ");
    App_Dbg_FmtFloat(num, yaw_show, 1U);
    p = dbg_stpcpy(p, num);
    if (App_Debug_GetStep() == DEBUG_STEP_S10
        || App_Debug_GetStep() == DEBUG_STEP_S11
        || App_Debug_GetStep() == DEBUG_STEP_S12
        || App_Debug_GetStep() == DEBUG_STEP_S14)
    {
        p = dbg_stpcpy(p, " W");
        App_Dbg_FmtFloat(num, v.w_cmd_degps, 1U);
        p = dbg_stpcpy(p, num);
    }
    *p = '\0';
    Oled_DrawStrLine(1U, line);

    p = line;
    if (App_Debug_GetStep() == DEBUG_STEP_S14)
    {
        uint16_t idx = 0U;
        uint16_t total = 0U;
        Plan_State_e ps = Chassis_Path_GetState(&idx, &total);

        p = dbg_stpcpy(p, "SEG ");
        App_Dbg_FmtInt(num, (int32_t)idx);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, "/");
        App_Dbg_FmtInt(num, (int32_t)total);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, (ps == PLAN_RUNNING) ? " RUN"
                      : (ps == PLAN_DONE)    ? " END"
                      : (ps == PLAN_ERROR)   ? " ERR" : " IDL");
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S12)
    {
        /* S12 三路航向对照：0x53 融合 yaw / wz 积分 / 编码器差速 */
        p = dbg_stpcpy(p, "GY ");
        App_Dbg_FmtFloat(num, v.yaw_gyro_deg, 1U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S11)
    {
        p = dbg_stpcpy(p, "TA ");
        App_Dbg_FmtFloat(num, s_dbg_angle, 0U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S10)
    {
        p = dbg_stpcpy(p, "TD ");
        App_Dbg_FmtFloat(num, s_dbg_dist, 0U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S9)
    {
        p = dbg_stpcpy(p, "TL ");
        App_Dbg_FmtFloat(num, s_dbg_tl, 0U);
    }
    else
    {
        p = dbg_stpcpy(p, "EL ");
        App_Dbg_FmtInt(num, pulse_l);
    }
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(2U, line);

    p = line;
    if (App_Debug_GetStep() == DEBUG_STEP_S12)
    {
        p = dbg_stpcpy(p, "WZ ");
        App_Dbg_FmtFloat(num, v.yaw_wz_deg, 1U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S10)
    {
        p = dbg_stpcpy(p, "TV ");
        App_Dbg_FmtFloat(num, s_dbg_speed, 0U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S9)
    {
        p = dbg_stpcpy(p, "TR ");
        App_Dbg_FmtFloat(num, s_dbg_tr, 0U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S11)
    {
        /* 纯陀螺航向（无编码器融合），与 Line1 的融合航向对照 */
        p = dbg_stpcpy(p, "GY ");
        App_Dbg_FmtFloat(num, v.yaw_gyro_deg, 1U);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S14)
    {
        /* 演示流程诊断：状态 + 机械臂忙标志 */
        p = dbg_stpcpy(p, "FLW ");
        App_Dbg_FmtInt(num, (int32_t)s_full_state);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, " AB");
        App_Dbg_FmtInt(num, (int32_t)Arm_IsBusy());
        p = dbg_stpcpy(p, num);
        num[0] = '\0';   /* 统一追加时不再重复 */
    }
    else
    {
        p = dbg_stpcpy(p, "ER ");
        App_Dbg_FmtInt(num, pulse_r);
    }
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(3U, line);

    p = line;
    if (App_Debug_GetStep() == DEBUG_STEP_S12)
    {
        p = dbg_stpcpy(p, "EN ");
        App_Dbg_FmtFloat(num, v.yaw_enc_deg, 1U);
    }
    else
    {
        p = dbg_stpcpy(p, "VL ");
        App_Dbg_FmtFloat(num, v.v_l_mmps, 0U);
    }
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(4U, line);

    p = line;
    p = dbg_stpcpy(p, "VR ");
    App_Dbg_FmtFloat(num, v.v_r_mmps, 0U);
    p = dbg_stpcpy(p, num);
    *p = '\0';
    Oled_DrawStrLine(5U, line);

    p = line;
    p = dbg_stpcpy(p, "DST ");
    App_Dbg_FmtFloat(num, v.dist_mm, 0U);
    p = dbg_stpcpy(p, num);
    p = dbg_stpcpy(p, " MM");
    *p = '\0';
    Oled_DrawStrLine(6U, line);

    p = line;
    if (App_Debug_GetStep() == DEBUG_STEP_S3)
    {
        /* 帧率（合法帧/秒，0x52+0x53 合计）与角速度 */
        p = dbg_stpcpy(p, "R");
        App_Dbg_FmtInt(num, (int32_t)s_frame_rate);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, " W");
        App_Dbg_FmtFloat(num, v.wz_dps, 1U);
        p = dbg_stpcpy(p, num);
    }
    else if (App_Debug_GetStep() == DEBUG_STEP_S14)
    {
        /* 演示流程诊断：四轴在线状态 + 机械臂错误码 */
        Arm_Debug_View_t av;

        Arm_Debug_GetView(&av);
        p = dbg_stpcpy(p, "ON ");
        *p++ = av.axis_online[ARM_AXIS_J1] ? '1' : '-';
        *p++ = av.axis_online[ARM_AXIS_J2] ? '2' : '-';
        *p++ = av.axis_online[ARM_AXIS_J3] ? '3' : '-';
        *p++ = av.axis_online[ARM_AXIS_J4] ? '4' : '-';
        p = dbg_stpcpy(p, " E");
        App_Dbg_FmtInt(num, (int32_t)Arm_GetLastError());
        p = dbg_stpcpy(p, num);
        num[0] = '\0';
    }
    else
    {
        p = dbg_stpcpy(p, "PWM ");
        App_Dbg_FmtInt(num, (int32_t)v.pwm_l);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, "/");
        App_Dbg_FmtInt(num, (int32_t)v.pwm_r);
        p = dbg_stpcpy(p, num);
        p = dbg_stpcpy(p, (v.gyro_online != 0U) ? " G" : " L");
    }
    *p = '\0';
    Oled_DrawStrLine(7U, line);

    Oled_Flush();
}
