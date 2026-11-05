/**
  ******************************************************************************
  * @file    chassis/control/ctrl.c
  * @brief   底盘控制层实现。
  *
  * 控制结构：
  *   STRAIGHT : 航向环(外) -> 左右轮速度环(内)
  *   SPIN     : 转角环(外) -> 差速速度环(内)
  *   ARC      : 运动学前馈 -> 速度环(内)，以陀螺仪转角收尾
  *
  * 到点顺序严格为：减速 -> 到位判定 -> 反向脉冲 -> 短路制动。
  ******************************************************************************
  */

#include "chassis/control/ctrl.h"
#include "chassis/chassis_config.h"
#include "chassis/control/kin.h"
#include "chassis/driver/gyro.h"
#include "chassis/driver/motor.h"
#include "chassis/perception/odom.h"
#include "common/math_util.h"
#include "common/pid.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define CTRL_CMD_QUEUE_LEN   (4U)
#define CTRL_DONE_QUEUE_LEN  (4U)

static osMessageQueueId_t s_q_cmd;
static osMessageQueueId_t s_q_done;

static uint8_t s_cmd_cb[128] __attribute__((aligned(8)));
static uint8_t s_done_cb[128] __attribute__((aligned(8)));
static uint8_t s_cmd_mq[CTRL_CMD_QUEUE_LEN * sizeof(Chassis_Cmd_t)] __attribute__((aligned(8)));
static uint8_t s_done_mq[CTRL_DONE_QUEUE_LEN * sizeof(Chassis_Status_t)] __attribute__((aligned(8)));

static Chassis_Cmd_t    s_cmd;
static Chassis_Status_t s_telem;

static Pid_t s_pid_heading;
static Pid_t s_pid_rate;       /* 航向内环：角速度环 */
static Pid_t s_pid_spin;
static Pid_t s_pid_vl;
static Pid_t s_pid_vr;

static float   s_planned_heading;
static float   s_last_w_cmd;
static float   s_spin_w_prev;      /* 旋转环上一拍输出，用于角加速度限制 */
static float   s_arc_cx;           /* 圆弧圆心（进入段时一次计算） */
static float   s_arc_cy;
static float   s_arc_r;
static float   s_arc_dir;          /* +1 左转 / -1 右转 */
static uint8_t s_arc_valid;
static float   s_dist_start;       /* 本段起点处的累计里程，用于相对距离判定 */
static float   s_vl_prev;          /* 上一拍左右目标轮速，用于变化率限制 */
static float   s_vr_prev;
static Chassis_Mode_e s_last_mode; /* 上一段模式，用于判断是否需要复位航向积分 */
static uint8_t s_path_anchored;

static uint8_t  s_active;
static uint8_t  s_debug_override;
static uint8_t  s_debug_speed;
static float    s_debug_v_l;
static float    s_debug_v_r;
static uint16_t s_debug_seq;
static uint8_t  s_braking;
static uint8_t  s_arrived;
static uint32_t s_brake_start_ms;
static uint32_t s_arrive_ms;
static float    s_v_prev_mag;
static uint32_t s_stall_start_ms;  /* 本轮「疑似堵转」首次成立的时刻 */
static uint8_t  s_stall_live;      /* 1 = 当前处于疑似堵转计时中 */

static void ctrl_pid_setup(Pid_t *pid, float kp, float ki, float kd,
                           float out_lim, float i_lim,
                           float d_alpha, float deadband, float i_sep)
{
    Pid_Init(pid, kp, ki, -out_lim, out_lim);
    Pid_SetGains(pid, kp, ki, kd);
    Pid_SetLimits(pid, -out_lim, out_lim, -i_lim, i_lim);
    Pid_SetFilter(pid, d_alpha, deadband, i_sep);
}

void Chassis_Ctrl_Init(void)
{
    s_q_cmd = NULL;
    s_q_done = NULL;

    memset(&s_cmd, 0, sizeof(s_cmd));
    memset(&s_telem, 0, sizeof(s_telem));

    ctrl_pid_setup(&s_pid_heading,
                   CHASSIS_PID_H_KP, CHASSIS_PID_H_KI, CHASSIS_PID_H_KD,
                   CHASSIS_W_MAX_DPS, CHASSIS_PID_H_I_LIMIT,
                   CHASSIS_PID_H_D_ALPHA, CHASSIS_PID_H_DEADBAND_DEG,
                   CHASSIS_PID_H_I_SEP_DEG);

    ctrl_pid_setup(&s_pid_rate,
                   CHASSIS_PID_RATE_KP, CHASSIS_PID_RATE_KI,
                   CHASSIS_PID_RATE_KD,
                   CHASSIS_W_MAX_DPS, CHASSIS_PID_RATE_I_LIMIT,
                   1.0f, 0.0f, 0.0f);

    ctrl_pid_setup(&s_pid_spin,
                   CHASSIS_PID_S_KP, CHASSIS_PID_S_KI, CHASSIS_PID_S_KD,
                   CHASSIS_SPIN_W_MAX_DPS, CHASSIS_PID_S_I_LIMIT,
                   CHASSIS_PID_S_D_ALPHA, CHASSIS_PID_S_DEADBAND_DEG,
                   CHASSIS_PID_S_I_SEP_DEG);

    ctrl_pid_setup(&s_pid_vl,
                   CHASSIS_PID_VL_KP, CHASSIS_PID_VL_KI, CHASSIS_PID_VL_KD,
                   (float)CHASSIS_PWM_MAX, CHASSIS_PID_V_I_LIMIT,
                   CHASSIS_PID_V_D_ALPHA, CHASSIS_PID_V_DEADBAND_MMPS,
                   CHASSIS_PID_V_I_SEP_MMPS);

    ctrl_pid_setup(&s_pid_vr,
                   CHASSIS_PID_VR_KP, CHASSIS_PID_VR_KI, CHASSIS_PID_VR_KD,
                   (float)CHASSIS_PWM_MAX, CHASSIS_PID_V_I_LIMIT,
                   CHASSIS_PID_V_D_ALPHA, CHASSIS_PID_V_DEADBAND_MMPS,
                   CHASSIS_PID_V_I_SEP_MMPS);

    s_planned_heading = 0.0f;
    s_path_anchored = 0U;
    s_active = 0U;
    s_debug_override = 0U;
    s_debug_speed = 0U;
    s_debug_v_l = 0.0f;
    s_debug_v_r = 0.0f;
    s_dist_start = 0.0f;
    s_vl_prev = 0.0f;
    s_vr_prev = 0.0f;
    s_last_mode = CHASSIS_MODE_STOP;
    s_braking = 0U;
    s_arrived = 0U;
    s_v_prev_mag = 0.0f;
}

void Chassis_Ctrl_InitRtos(void)
{
    osMessageQueueAttr_t attr;

    memset(&attr, 0, sizeof(attr));
    attr.name = "chas_cmd";
    attr.cb_mem = s_cmd_cb;
    attr.cb_size = sizeof(s_cmd_cb);
    attr.mq_mem = s_cmd_mq;
    attr.mq_size = sizeof(s_cmd_mq);
    s_q_cmd = osMessageQueueNew(CTRL_CMD_QUEUE_LEN, sizeof(Chassis_Cmd_t), &attr);

    memset(&attr, 0, sizeof(attr));
    attr.name = "chas_done";
    attr.cb_mem = s_done_cb;
    attr.cb_size = sizeof(s_done_cb);
    attr.mq_mem = s_done_mq;
    attr.mq_size = sizeof(s_done_mq);
    s_q_done = osMessageQueueNew(CTRL_DONE_QUEUE_LEN, sizeof(Chassis_Status_t), &attr);
}

void Chassis_Ctrl_BeginPath(void)
{
    Chassis_Odom_State_t odom;

    Chassis_Odom_GetState(&odom);
    s_planned_heading = odom.yaw_deg;
    s_path_anchored = 1U;
    s_last_mode = CHASSIS_MODE_STOP;   /* 强制首段复位航向环 */
    Pid_Reset(&s_pid_heading);
}

void Chassis_Ctrl_Submit(const Chassis_Cmd_t *cmd, uint8_t reset_integral)
{
    if (cmd == NULL)
    {
        return;
    }

    if (s_path_anchored == 0U)
    {
        Chassis_Ctrl_BeginPath();
    }

    /* 绝对航向在控制层内部累加，规划层只提供相对量 */
    s_planned_heading += cmd->heading_delta_deg;

    if (reset_integral != 0U)
    {
        Pid_Reset(&s_pid_vl);
        Pid_Reset(&s_pid_vr);
        Pid_Reset(&s_pid_heading);
        Pid_Reset(&s_pid_spin);
    }

    if (s_q_cmd != NULL)
    {
        (void)osMessageQueuePut(s_q_cmd, cmd, 0U, 0U);
    }
}

/* 左右轮目标速度的变化率限制：直道↔弯道的曲率突变不再直接阶跃到电机 */
static void ctrl_limit_wheel_ramp(float *v_l, float *v_r, float dt_s)
{
    float max_dv = CHASSIS_WHEEL_ACCEL_MMPS2 * dt_s;

    *v_l = Math_Clamp(*v_l, s_vl_prev - max_dv, s_vl_prev + max_dv);
    *v_r = Math_Clamp(*v_r, s_vr_prev - max_dv, s_vr_prev + max_dv);

    s_vl_prev = *v_l;
    s_vr_prev = *v_r;
}

/* 速度环输出 = PID 残差 + 前馈。前馈随目标速度线性放大，
   高速时不需要重新整定 PID。 */
static int16_t ctrl_speed_output(Pid_t *pid, float target_mmps,
                                 float meas_mmps, float dt_s, float kff)
{
    float out = Pid_Update(pid, target_mmps, meas_mmps, dt_s)
              + kff * target_mmps;

    return (int16_t)Math_Clamp(out,
                               (float)(-CHASSIS_PWM_MAX),
                               (float)(CHASSIS_PWM_MAX));
}

static void ctrl_publish_status(const Chassis_Odom_State_t *odom,
                                uint8_t finished, uint8_t fault)
{
    taskENTER_CRITICAL();
    s_telem.seq = s_cmd.seq;
    s_telem.mode = s_cmd.mode;
    s_telem.finished = finished;
    s_telem.fault = fault;
    s_telem.v_meas_mmps = odom->v_mmps;
    s_telem.w_meas_degps = odom->w_degps;
    s_telem.w_cmd_degps = s_last_w_cmd;
    s_telem.heading_meas_deg = odom->yaw_deg;
    s_telem.dist_done_mm = odom->ds_seg_mm - s_dist_start;
    s_telem.pwm_l = Chassis_Motor_GetOutput(CHASSIS_MOTOR_LEFT);
    s_telem.pwm_r = Chassis_Motor_GetOutput(CHASSIS_MOTOR_RIGHT);
    taskEXIT_CRITICAL();
}

static void ctrl_finish(const Chassis_Odom_State_t *odom, uint8_t fault)
{
    Chassis_Status_t st;

    st.seq = s_cmd.seq;
    st.mode = s_cmd.mode;
    st.finished = 1U;
    st.fault = fault;
    st.v_meas_mmps = odom->v_mmps;
    st.w_meas_degps = odom->w_degps;
    st.heading_meas_deg = odom->yaw_deg;
    st.dist_done_mm = odom->ds_seg_mm - s_dist_start;
    st.pwm_l = Chassis_Motor_GetOutput(CHASSIS_MOTOR_LEFT);
    st.pwm_r = Chassis_Motor_GetOutput(CHASSIS_MOTOR_RIGHT);

    ctrl_publish_status(odom, 1U, fault);

    if (s_q_done != NULL)
    {
        (void)osMessageQueuePut(s_q_done, &st, 0U, 0U);
    }

    s_active = 0U;
    s_braking = 0U;
    s_arrived = 0U;

    /* 完成后保持制动，避免惯性沿切向继续滑行 */
    Chassis_Motor_StopAll(1U);
}

/* 单轮堵转判据：目标轮速明显、实测几乎为零、占空比已饱和。
   没有电流采样，只能用这条纯软件判据（见审计 H13）。 */
static uint8_t ctrl_wheel_stuck(float target_mmps, float meas_mmps, int16_t pwm)
{
    float mag = fabsf(target_mmps);
    int16_t apwm = (pwm < 0) ? (int16_t)(-pwm) : pwm;

    if (mag < CHASSIS_STALL_MIN_TARGET_MMPS)
    {
        return 0U;   /* 目标太小：起步惯性、末端收速度都可能符合「不动」 */
    }
    if (fabsf(meas_mmps) >= (CHASSIS_STALL_SPEED_RATIO * mag))
    {
        return 0U;   /* 轮子确实在转 */
    }
    if ((float)apwm < (CHASSIS_STALL_PWM_RATIO * (float)CHASSIS_PWM_MAX))
    {
        return 0U;   /* 还没顶到饱和：只是加速中或负载偏大 */
    }

    return 1U;
}

/* 堵转计时与处置。返回 1 表示本段已按故障结束（电机已停，本拍不要下发 PWM）。
   闭环在轮子被挡住时会把占空比顶到 ±CHASSIS_PWM_MAX 一动不动，
   以前只能等段超时（1700mm 段约 25s）才结束。 */
static uint8_t ctrl_stall_service(float v_l_t, float v_r_t,
                                  int16_t pwm_l, int16_t pwm_r,
                                  const Chassis_Odom_State_t *odom)
{
    uint32_t now = HAL_GetTick();

    if (ctrl_wheel_stuck(v_l_t, odom->v_l_mmps, pwm_l) == 0U
        && ctrl_wheel_stuck(v_r_t, odom->v_r_mmps, pwm_r) == 0U)
    {
        s_stall_live = 0U;
        return 0U;
    }

    if (s_stall_live == 0U)
    {
        s_stall_live = 1U;
        s_stall_start_ms = now;
        return 0U;
    }

    if ((now - s_stall_start_ms) < CHASSIS_STALL_HOLD_MS)
    {
        return 0U;
    }

    s_stall_live = 0U;
    ctrl_finish(odom, 1U);
    return 1U;
}

static void ctrl_arrival(const Chassis_Odom_State_t *odom)
{
    s_arrived = 1U;
    s_arrive_ms = HAL_GetTick();

    if (s_cmd.brake_pulse != 0U
        && (fabsf(odom->v_l_mmps) > CHASSIS_V_MIN_MMPS
            || fabsf(odom->v_r_mmps) > CHASSIS_V_MIN_MMPS))
    {
        s_braking = 1U;
        s_brake_start_ms = HAL_GetTick();
    }
}

/* 按当前运动模式返回 (最大制动 PWM, 脉冲时长) */
static void ctrl_brake_params(int16_t *pwm_max, uint32_t *pulse_ms)
{
    switch (s_cmd.mode)
    {
    case CHASSIS_MODE_ARC:
        *pwm_max = CHASSIS_ARC_BRAKE_PWM_MAX;
        *pulse_ms = CHASSIS_ARC_BRAKE_PULSE_MS;
        break;

    case CHASSIS_MODE_SPIN:
        *pwm_max = CHASSIS_SPIN_BRAKE_PWM_MAX;
        *pulse_ms = CHASSIS_SPIN_BRAKE_PULSE_MS;
        break;

    case CHASSIS_MODE_STRAIGHT:
    default:
        *pwm_max = CHASSIS_STRAIGHT_BRAKE_PWM_MAX;
        *pulse_ms = CHASSIS_STRAIGHT_BRAKE_PULSE_MS;
        break;
    }
}

static void ctrl_apply_brake_pulse(const Chassis_Odom_State_t *odom)
{
    float vmag_l = fabsf(odom->v_l_mmps);
    float vmag_r = fabsf(odom->v_r_mmps);
    float vmag = (vmag_l > vmag_r) ? vmag_l : vmag_r;
    int16_t pwm_max;
    uint32_t pulse_ms;
    int16_t bp;
    int16_t pwm_l;
    int16_t pwm_r;

    ctrl_brake_params(&pwm_max, &pulse_ms);
    (void)pulse_ms;

    bp = (int16_t)Math_Clamp(vmag * CHASSIS_BRAKE_PWM_SCALE,
                             (float)CHASSIS_BRAKE_PWM_MIN,
                             (float)pwm_max);

    pwm_l = (odom->v_l_mmps >= 0.0f) ? (int16_t)(-bp) : bp;
    pwm_r = (odom->v_r_mmps >= 0.0f) ? (int16_t)(-bp) : bp;

    Chassis_Motor_SetOutput(CHASSIS_MOTOR_LEFT, pwm_l);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_RIGHT, pwm_r);
}

static void ctrl_run_straight(const Chassis_Odom_State_t *odom,
                              float dt_s, float *v_l, float *v_r)
{
    float sgn = (s_cmd.dist_mm >= 0.0f) ? 1.0f : -1.0f;
    float target_mag = fabsf(s_cmd.dist_mm);
    float progress = (odom->ds_seg_mm - s_dist_start) * sgn;
    float remaining = target_mag - progress;
    float v_allow;
    float v_ramp;
    float v_mag;
    float w_cmd;
    float end_v;

    if (remaining < 0.0f)
    {
        remaining = 0.0f;
    }

    /* 段末目标速度：非 0 时速度穿过段边界，不再每段归零 */
    end_v = fabsf(s_cmd.end_speed_mmps);

    /* 减速段：随剩余距离收缩允许速度；巡航段由加速度斜坡爬升 */
    v_allow = sqrtf(2.0f * CHASSIS_A_DECEL_MMPS2 * remaining
                    + end_v * end_v);
    v_ramp = s_v_prev_mag + CHASSIS_A_ACCEL_MMPS2 * dt_s;
    v_mag = fabsf(s_cmd.v_mmps);

    if (v_allow < v_mag)
    {
        v_mag = v_allow;
    }
    if (v_ramp < v_mag)
    {
        v_mag = v_ramp;
    }
    if (v_mag < 0.0f)
    {
        v_mag = 0.0f;
    }

    s_v_prev_mag = v_mag;

    /* 串级：外环角度 -> 目标角速度；内环角速度 -> 实际 ω 指令。
       内环积分直接补偿打滑造成的转向损失，不再依赖外环累积误差。 */
    {
        float w_target = Pid_Update(&s_pid_heading,
                                    s_planned_heading, odom->yaw_deg, dt_s);

        w_cmd = Pid_Update(&s_pid_rate,
                           w_target, odom->yaw_rate_dps, dt_s);
    }

    s_last_w_cmd = w_cmd;
    Chassis_Kin_Inverse(v_mag * sgn, w_cmd, v_l, v_r);

    if (remaining <= CHASSIS_POS_TOL_MM)
    {
        ctrl_arrival(odom);
    }
}

static void ctrl_run_spin(const Chassis_Odom_State_t *odom,
                          float dt_s, float *v_l, float *v_r)
{
    float w_cmd;
    float ang_err;
    float w_allow;
    float dw_max;

    ang_err = Math_Wrap180(s_planned_heading - odom->yaw_deg);
    w_cmd = Pid_Update(&s_pid_spin, s_planned_heading, odom->yaw_deg, dt_s);

    /* 1) 减速曲线：按剩余角度限制角速度，避免冲过头 */
    w_allow = sqrtf(2.0f * CHASSIS_ALPHA_MAX_DPS2 * fabsf(ang_err));
    if (fabsf(w_cmd) > w_allow)
    {
        w_cmd = (w_cmd > 0.0f) ? w_allow : -w_allow;
    }

    /* 2) 最小角速度优先：必须放在减速曲线之后，否则会被压回去。
          末端保持足够转速，保证右轮越过静摩擦死区。 */
    if (fabsf(ang_err) > CHASSIS_SPIN_DEAD_ZONE_DEG
        && fabsf(w_cmd) < CHASSIS_SPIN_W_MIN)
    {
        w_cmd = (ang_err > 0.0f) ? CHASSIS_SPIN_W_MIN : -CHASSIS_SPIN_W_MIN;
    }

    /* 3) 角加速度限制：不同负载下角速度剖面一致 */
    dw_max = CHASSIS_ALPHA_MAX_DPS2 * dt_s;
    if (w_cmd > (s_spin_w_prev + dw_max))
    {
        w_cmd = s_spin_w_prev + dw_max;
    }
    else if (w_cmd < (s_spin_w_prev - dw_max))
    {
        w_cmd = s_spin_w_prev - dw_max;
    }
    s_spin_w_prev = w_cmd;

    s_last_w_cmd = w_cmd;
    Chassis_Kin_Spin(w_cmd, v_l, v_r);

    if (fabsf(ang_err) < CHASSIS_ANG_TOL_DEG)
    {
        ctrl_arrival(odom);
    }
}

static void ctrl_run_arc(const Chassis_Odom_State_t *odom,
                         float dt_s, float *v_l, float *v_r)
{
    float v_target;
    float v_cmd;
    float w_ff;
    float w_cmd;
    float ang_err;
    float rem_rad;
    float v_allow;
    float dx;
    float dy;
    float r_meas;
    float e_r;

    (void)dt_s;

    ang_err = Math_Wrap180(s_planned_heading - odom->yaw_deg);

    if (fabsf(s_cmd.radius_mm) < 1.0f)
    {
        /* 半径退化：按原地旋转处理 */
        Chassis_Kin_Arc(fabsf(s_cmd.v_mmps), s_cmd.radius_mm, v_l, v_r);
        s_last_w_cmd = 0.0f;
    }
    else
    {
        /* 圆弧减速规划：按剩余角度收缩目标线速度，避免外侧轮惯性冲过头。
           审计 H12：这里的 v 只是弧长方向的线速度大小，转向完全由
           radius_mm 的符号（经 w_ff）决定。旧代码直接取 s_cmd.v_mmps，
           dir<0 的右弧拿到负速度，下面的最小速度钳位把负值翻成正的
           CHASSIS_ARC_V_MIN_MMPS ⇒ v_allow < v_target 恒不成立，
           右弧既没有减速规划、速度还被动降到最小速度。 */
        v_target = fabsf(s_cmd.v_mmps);

        if (s_arc_valid != 0U)
        {
            rem_rad = fabsf(ang_err) * MATH_DEG2RAD;
            v_allow = sqrtf(2.0f * CHASSIS_A_DECEL_MMPS2 * rem_rad * s_arc_r
                            + fabsf(s_cmd.end_speed_mmps) * fabsf(s_cmd.end_speed_mmps));

            if (v_allow < v_target)
            {
                v_target = v_allow;
            }
            if (v_target < CHASSIS_ARC_V_MIN_MMPS)
            {
                v_target = CHASSIS_ARC_V_MIN_MMPS;
            }
        }

        /* 第 1 层：标定补偿（左右分别标定） */
        v_cmd = v_target * ((s_cmd.radius_mm >= 0.0f)
                          ? CHASSIS_ARC_V_SCALE_L
                          : CHASSIS_ARC_V_SCALE_R);
        w_ff = (v_target / s_cmd.radius_mm)
             * CHASSIS_ARC_W_SCALE * MATH_RAD2DEG;

        /* 第 2 层：径向比例反馈（只修残差） */
        w_cmd = w_ff;
        if (s_arc_valid != 0U)
        {
            dx = odom->x_mm - s_arc_cx;
            dy = odom->y_mm - s_arc_cy;
            r_meas = sqrtf(dx * dx + dy * dy);
            e_r = r_meas - s_arc_r;
            w_cmd = w_ff + CHASSIS_ARC_KR * e_r * s_arc_dir;
        }

        w_cmd = Math_Clamp(w_cmd, -CHASSIS_W_MAX_DPS, CHASSIS_W_MAX_DPS);
        s_last_w_cmd = w_cmd;

        Chassis_Kin_Inverse(v_cmd, w_cmd, v_l, v_r);
    }

    /* 预测式提前触发：按当前角速度预估滑行量，提前进入制动 */
    {
        float lead_deg = fabsf(odom->w_degps) * CHASSIS_ARC_BRAKE_LEAD_S;

        if (lead_deg > CHASSIS_ARC_BRAKE_LEAD_MAX_DEG)
        {
            lead_deg = CHASSIS_ARC_BRAKE_LEAD_MAX_DEG;
        }

        if (fabsf(ang_err) < (CHASSIS_ANG_TOL_DEG + lead_deg))
        {
            ctrl_arrival(odom);
        }
    }
}

void Chassis_Ctrl_Update(void)
{
    Chassis_Odom_State_t odom;
    Chassis_Cmd_t cmd;
    float dt_s = (float)CHASSIS_CTRL_PERIOD_MS * 0.001f;
    float v_l_t = 0.0f;
    float v_r_t = 0.0f;
    int16_t pwm_l;
    int16_t pwm_r;
    int16_t brake_pwm_max;
    uint32_t brake_pulse_ms;

    /* 调试接管期间控制环完全不碰电机 */
    if (s_debug_override != 0U)
    {
        return;
    }

    Chassis_Odom_GetState(&odom);

    /* S9 调试速度环：复用正式 PID，左右独立给定 */
    if (s_debug_speed != 0U)
    {
        pwm_l = ctrl_speed_output(&s_pid_vl, s_debug_v_l,
                                  odom.v_l_mmps, dt_s, CHASSIS_SPEED_FF_L);
        pwm_r = ctrl_speed_output(&s_pid_vr, s_debug_v_r,
                                  odom.v_r_mmps, dt_s, CHASSIS_SPEED_FF_R);
        Chassis_Motor_SetOutput(CHASSIS_MOTOR_LEFT, pwm_l);
        Chassis_Motor_SetOutput(CHASSIS_MOTOR_RIGHT, pwm_r);
        ctrl_publish_status(&odom, 0U, 0U);
        return;
    }

    /* 取新指令（非阻塞）。段间切换不经过 Stop，速度环积分保持 */
    if (s_q_cmd != NULL
        && osMessageQueueGet(s_q_cmd, &cmd, NULL, 0U) == osOK)
    {
        s_cmd = cmd;
        s_active = 1U;
        s_braking = 0U;
        s_arrived = 0U;
        s_arrive_ms = 0U;
        s_v_prev_mag = fabsf(odom.v_mmps);
        s_vl_prev = odom.v_l_mmps;
        s_vr_prev = odom.v_r_mmps;

        /* 记录本段起点。用相对增量判定，避免依赖感知层清零的时机：
           MarkSegStart 是下一拍才生效的，若直接用 ds_seg_mm，新段第一拍
           会读到上一段的累计值，导致短段被立即判定为完成。 */
        s_dist_start = odom.ds_seg_mm;

        /* 速度环在段间保持积分以实现连贯。
           航向环只在模式变化时复位：直行段之间保持积分，避免每段从零重建，
           否则短直行段来不及积累出持续修正量。 */
        if (s_cmd.mode == CHASSIS_MODE_SPIN)
        {
            Pid_Reset(&s_pid_spin);
            s_spin_w_prev = 0.0f;
        }

        if (s_cmd.mode != s_last_mode)
        {
            Pid_Reset(&s_pid_heading);
            s_last_mode = s_cmd.mode;
        }

        /* 内环每段复位：它跟踪的是瞬时角速度，段间不连续 */
        Pid_Reset(&s_pid_rate);

        /* ARC 圆心：必须用进入时的实际航向，不能用 planned_heading */
        if (s_cmd.mode == CHASSIS_MODE_ARC
            && fabsf(s_cmd.radius_mm) > 1.0f)
        {
            float th0 = odom.yaw_deg * MATH_DEG2RAD;

            s_arc_dir = (s_cmd.radius_mm >= 0.0f) ? 1.0f : -1.0f;
            s_arc_r = fabsf(s_cmd.radius_mm);
            s_arc_cx = odom.x_mm - s_arc_dir * s_arc_r * sinf(th0);
            s_arc_cy = odom.y_mm + s_arc_dir * s_arc_r * cosf(th0);
            s_arc_valid = 1U;
        }
        else
        {
            s_arc_valid = 0U;
        }
    }

    if (s_active == 0U)
    {
        /* 待机保持短路制动，防止惯性滑行；静止时无电流，不发热 */
        Chassis_Motor_StopAll(1U);
        return;
    }

    /* 需要航向反馈的模式在陀螺仪掉线时直接上报故障 */
    if ((s_cmd.mode == CHASSIS_MODE_SPIN || s_cmd.mode == CHASSIS_MODE_ARC)
        && Chassis_Gyro_IsOnline() == 0U)
    {
        ctrl_finish(&odom, 1U);
        return;
    }

    /* 阶段 4：反向脉冲制动 */
    if (s_braking != 0U)
    {
        ctrl_brake_params(&brake_pwm_max, &brake_pulse_ms);
        (void)brake_pwm_max;

        if ((HAL_GetTick() - s_brake_start_ms) < brake_pulse_ms)
        {
            ctrl_apply_brake_pulse(&odom);
            ctrl_publish_status(&odom, 0U, 0U);
            return;
        }

        s_braking = 0U;
        Pid_Reset(&s_pid_vl);
        Pid_Reset(&s_pid_vr);
        Chassis_Motor_Brake(CHASSIS_MOTOR_LEFT);
        Chassis_Motor_Brake(CHASSIS_MOTOR_RIGHT);
        ctrl_publish_status(&odom, 0U, 0U);
        return;
    }

    /* 到位后等待停稳 */
    if (s_arrived != 0U)
    {
        Chassis_Motor_Brake(CHASSIS_MOTOR_LEFT);
        Chassis_Motor_Brake(CHASSIS_MOTOR_RIGHT);
        ctrl_publish_status(&odom, 0U, 0U);

        if ((HAL_GetTick() - s_arrive_ms) >= CHASSIS_ARRIVE_SETTLE_MS)
        {
            ctrl_finish(&odom, 0U);
        }
        return;
    }

    switch (s_cmd.mode)
    {
    case CHASSIS_MODE_STRAIGHT:
        ctrl_run_straight(&odom, dt_s, &v_l_t, &v_r_t);
        break;

    case CHASSIS_MODE_SPIN:
        ctrl_run_spin(&odom, dt_s, &v_l_t, &v_r_t);
        break;

    case CHASSIS_MODE_ARC:
        ctrl_run_arc(&odom, dt_s, &v_l_t, &v_r_t);
        break;

    case CHASSIS_MODE_SPEED_RAW:
        Chassis_Kin_Inverse(s_cmd.v_mmps, 0.0f, &v_l_t, &v_r_t);
        break;

    case CHASSIS_MODE_STOP:
    default:
        ctrl_finish(&odom, 0U);
        return;
    }

    if (s_arrived != 0U)
    {
        /* 本拍刚判定到位，转由下一拍处理制动 */
        return;
    }

    /* 轮速变化率限制：平滑直道↔弯道的曲率突变 */
    ctrl_limit_wheel_ramp(&v_l_t, &v_r_t, dt_s);

    pwm_l = ctrl_speed_output(&s_pid_vl, v_l_t,
                              odom.v_l_mmps, dt_s, CHASSIS_SPEED_FF_L);
    pwm_r = ctrl_speed_output(&s_pid_vr, v_r_t,
                              odom.v_r_mmps, dt_s, CHASSIS_SPEED_FF_R);

    /* 审计 H13：堵转保护。ctrl_finish 内部会上报 fault 并停车，本拍不再下发 */
    if (ctrl_stall_service(v_l_t, v_r_t, pwm_l, pwm_r, &odom) != 0U)
    {
        return;
    }

    Chassis_Motor_SetOutput(CHASSIS_MOTOR_LEFT, pwm_l);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_RIGHT, pwm_r);

    ctrl_publish_status(&odom, 0U, 0U);
}

Chassis_Wait_e Chassis_Ctrl_WaitDone(Chassis_Status_t *st, uint32_t timeout_ms)
{
    Chassis_Status_t tmp;

    if (s_q_done == NULL)
    {
        return CHASSIS_WAIT_TIMEOUT;
    }

    if (osMessageQueueGet(s_q_done, &tmp, NULL, timeout_ms) != osOK)
    {
        return CHASSIS_WAIT_TIMEOUT;
    }

    if (st != NULL)
    {
        *st = tmp;
    }

    return (tmp.fault != 0U) ? CHASSIS_WAIT_FAULT : CHASSIS_WAIT_DONE;
}

void Chassis_Ctrl_GetStatus(Chassis_Status_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_telem;
    taskEXIT_CRITICAL();
}

void Chassis_Ctrl_SetDebugOverride(uint8_t on)
{
    s_debug_override = (on != 0U) ? 1U : 0U;

    if (s_debug_override != 0U)
    {
        s_active = 0U;
        s_braking = 0U;
        s_arrived = 0U;
    }
}

void Chassis_Ctrl_DebugRunSpeed(float v_l_mmps, float v_r_mmps)
{
    s_debug_override = 0U;   /* 控制环必须运行 */
    s_active = 0U;
    s_braking = 0U;
    s_arrived = 0U;

    s_debug_v_l = v_l_mmps;
    s_debug_v_r = v_r_mmps;

    Pid_Reset(&s_pid_vl);
    Pid_Reset(&s_pid_vr);

    s_debug_speed = 1U;
}

void Chassis_Ctrl_DebugStopSpeed(void)
{
    s_debug_speed = 0U;
    s_debug_v_l = 0.0f;
    s_debug_v_r = 0.0f;

    Pid_Reset(&s_pid_vl);
    Pid_Reset(&s_pid_vr);

    Chassis_Motor_StopAll(1U);
}

void Chassis_Ctrl_DebugRunStraight(float dist_mm, float speed_mmps)
{
    Chassis_Cmd_t cmd;

    /* 退出速度调试模式，让正式串级控制接管 */
    s_debug_speed = 0U;
    s_debug_override = 0U;
    s_debug_v_l = 0.0f;
    s_debug_v_r = 0.0f;

    /* 每次直行都从当前航向重新锚定 */
    Chassis_Ctrl_BeginPath();

    cmd.mode = CHASSIS_MODE_STRAIGHT;
    cmd.v_mmps = speed_mmps;
    cmd.heading_delta_deg = 0.0f;
    cmd.radius_mm = 0.0f;
    cmd.dist_mm = dist_mm;
    cmd.brake_pulse = 1U;
    cmd.seg_last = 1U;
    cmd.seq = ++s_debug_seq;

    Chassis_Ctrl_Submit(&cmd, 1U);
}

void Chassis_Ctrl_DebugStopStraight(void)
{
    Chassis_Ctrl_Stop(1U);
}

void Chassis_Ctrl_DebugRunArc(float radius_mm, float angle_deg, float speed_mmps)
{
    Chassis_Cmd_t cmd;

    s_debug_speed = 0U;
    s_debug_override = 0U;
    s_debug_v_l = 0.0f;
    s_debug_v_r = 0.0f;

    Chassis_Ctrl_BeginPath();

    cmd.mode = CHASSIS_MODE_ARC;
    cmd.v_mmps = speed_mmps;
    cmd.heading_delta_deg = angle_deg;
    cmd.radius_mm = radius_mm;
    cmd.dist_mm = 0.0f;
    cmd.brake_pulse = 1U;
    cmd.seg_last = 1U;
    cmd.seq = ++s_debug_seq;

    Chassis_Ctrl_Submit(&cmd, 1U);
}

void Chassis_Ctrl_DebugRunSpin(float angle_deg)
{
    Chassis_Cmd_t cmd;

    s_debug_speed = 0U;
    s_debug_override = 0U;
    s_debug_v_l = 0.0f;
    s_debug_v_r = 0.0f;

    Chassis_Ctrl_BeginPath();

    cmd.mode = CHASSIS_MODE_SPIN;
    cmd.v_mmps = 0.0f;
    cmd.heading_delta_deg = angle_deg;
    cmd.radius_mm = 0.0f;
    cmd.dist_mm = 0.0f;
    cmd.brake_pulse = 1U;
    cmd.seg_last = 1U;
    cmd.seq = ++s_debug_seq;

    Chassis_Ctrl_Submit(&cmd, 1U);
}

void Chassis_Ctrl_Stop(uint8_t brake)
{
    s_debug_override = 0U;
    s_debug_speed = 0U;
    s_active = 0U;
    s_braking = 0U;
    s_arrived = 0U;
    s_stall_live = 0U;   /* 停止后清堵转计时，新指令重新开始计时 */

    if (s_q_cmd != NULL)
    {
        (void)osMessageQueueReset(s_q_cmd);
    }

    Chassis_Motor_StopAll(brake);
}
