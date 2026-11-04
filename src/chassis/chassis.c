/**
  ******************************************************************************
  * @file    chassis/chassis.c
  * @brief   底盘子系统统一入口与对外适配。
  ******************************************************************************
  */

#include "chassis/chassis.h"
#include "chassis/chassis_config.h"
#include "chassis/driver/encoder.h"
#include "chassis/driver/gyro.h"
#include "chassis/driver/motor.h"
#include "chassis/perception/odom.h"
#include "chassis/control/ctrl.h"
#include "chassis/planning/path.h"
#include "main.h"
#include "cmsis_os.h"

#include <stddef.h>

void Chassis_Init(void)
{
    Chassis_Gyro_Init();
    Chassis_Encoder_Init();
    Chassis_Motor_Init();
    Chassis_Odom_Init();
    Chassis_Ctrl_Init();
    Chassis_Path_Init();
}

void Chassis_InitRtos(void)
{
    Chassis_Ctrl_InitRtos();
}

void Chassis_CalibrateStart(void)
{
    Chassis_Gyro_StartBiasCalib((uint16_t)CHASSIS_GYRO_BIAS_SAMPLES);
}

uint8_t Chassis_CalibratePoll(void)
{
    return Chassis_Gyro_IsCalibDone();
}

int Chassis_Calibrate(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    Chassis_Gyro_StartBiasCalib(CHASSIS_GYRO_BIAS_SAMPLES);

    while (Chassis_Gyro_IsCalibDone() == 0U)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return -1;
        }
        osDelay(10);
    }

    return 0;
}

void Chassis_Sense_Update(void)
{
    Chassis_Odom_Update();
}

void Chassis_Control_Update(void)
{
    Chassis_Ctrl_Update();
}

void Chassis_Control_Stop(uint8_t brake)
{
    Chassis_Ctrl_Stop(brake);
}

void Chassis_ResetHeading(void)
{
    Chassis_Odom_Reset(0.0f, 0.0f, 0.0f);
}

void Chassis_Debug_SetMotor(int16_t left_pwm, int16_t right_pwm)
{
    Chassis_Ctrl_SetDebugOverride(1U);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_LEFT, left_pwm);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_RIGHT, right_pwm);
}

void Chassis_Debug_ReleaseMotor(void)
{
    Chassis_Motor_StopAll(0U);
    Chassis_Ctrl_SetDebugOverride(0U);
}

#define PULSE_RUN_TIMEOUT_MS   (15000U)

static int32_t  s_pulse_target;
static uint32_t s_pulse_start_ms;
static uint8_t  s_pulse_active;
static uint8_t  s_pulse_hold;

void Chassis_Debug_ResetPulses(void)
{
    Chassis_Encoder_ResetTotal(CHASSIS_ENC_LEFT);
    Chassis_Encoder_ResetTotal(CHASSIS_ENC_RIGHT);
}

void Chassis_Debug_GetPulses(int32_t *left, int32_t *right)
{
    if (left != NULL)
    {
        *left = Chassis_Encoder_GetTotal(CHASSIS_ENC_LEFT);
    }
    if (right != NULL)
    {
        *right = Chassis_Encoder_GetTotal(CHASSIS_ENC_RIGHT);
    }
}

void Chassis_Debug_RunSpeed(float left_mmps, float right_mmps)
{
    Chassis_Ctrl_DebugRunSpeed(left_mmps, right_mmps);
}

void Chassis_Debug_StopSpeed(void)
{
    Chassis_Ctrl_DebugStopSpeed();
}

void Chassis_Debug_RunStraight(float dist_mm, float speed_mmps)
{
    Chassis_Ctrl_DebugRunStraight(dist_mm, speed_mmps);
}

void Chassis_Debug_StopStraight(void)
{
    Chassis_Ctrl_DebugStopStraight();
}

void Chassis_Debug_RunSpin(float angle_deg)
{
    Chassis_Ctrl_DebugRunSpin(angle_deg);
}

void Chassis_Debug_RunArc(float radius_mm, float angle_deg, float speed_mmps)
{
    Chassis_Ctrl_DebugRunArc(radius_mm, angle_deg, speed_mmps);
}

void Chassis_Debug_RunPulses(int32_t target_pulses, int16_t pwm)
{
    Chassis_Debug_ResetPulses();

    s_pulse_target = target_pulses;
    s_pulse_start_ms = HAL_GetTick();
    s_pulse_active = 1U;
    s_pulse_hold = 0U;

    Chassis_Ctrl_SetDebugOverride(1U);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_LEFT, pwm);
    Chassis_Motor_SetOutput(CHASSIS_MOTOR_RIGHT, pwm);
}

void Chassis_Debug_StopPulses(void)
{
    s_pulse_active = 0U;
    s_pulse_hold = 0U;
    Chassis_Motor_StopAll(1U);
    Chassis_Ctrl_SetDebugOverride(0U);
}

void Chassis_Debug_Update(void)
{
    int32_t pl;
    int32_t pr;

    if (s_pulse_active != 0U)
    {
        pl = Chassis_Encoder_GetTotal(CHASSIS_ENC_LEFT);
        pr = Chassis_Encoder_GetTotal(CHASSIS_ENC_RIGHT);

        if (((pl + pr) / 2) >= s_pulse_target
            || (HAL_GetTick() - s_pulse_start_ms) > PULSE_RUN_TIMEOUT_MS)
        {
            s_pulse_active = 0U;
            s_pulse_hold = 1U;
        }
    }

    if (s_pulse_hold != 0U)
    {
        Chassis_Motor_StopAll(1U);   /* 到位后保持短路制动 */
    }
}

void Chassis_Debug_GetView(Chassis_Debug_View_t *out)
{
    Chassis_Odom_State_t odom;
    Chassis_Status_t st;
    Chassis_Gyro_Frame_t gy;

    if (out == NULL)
    {
        return;
    }

    Chassis_Odom_GetState(&odom);
    Chassis_Ctrl_GetStatus(&st);
    (void)Chassis_Gyro_GetFrame(&gy);

    out->heading_deg = odom.yaw_deg;
    out->yaw_gyro_deg = odom.yaw_gyro_deg;
    out->yaw_enc_deg = odom.yaw_enc_deg;
    out->yaw_wz_deg = odom.yaw_wz_deg;
    out->wz_dps = Chassis_Gyro_GetWzDps();
    out->dist_mm = st.dist_done_mm;   /* 本段相对距离，跨段不清零 */
    out->v_mmps = odom.v_mmps;
    out->v_l_mmps = odom.v_l_mmps;
    out->v_r_mmps = odom.v_r_mmps;
    out->w_degps = odom.w_degps;
    out->w_cmd_degps = st.w_cmd_degps;
    out->pwm_l = st.pwm_l;
    out->pwm_r = st.pwm_r;
    out->gyro_online = Chassis_Gyro_IsOnline();
    out->calib_done = Chassis_Gyro_IsCalibDone();
    out->gyro_bad_flag = gy.last_bad_flag;
    out->gyro_seq = gy.seq;
    out->gyro_rx_bytes = gy.rx_bytes;
    out->gyro_yaw_frames = gy.yaw_frames;
    out->gyro_wz_frames = gy.wz_frames;
    out->gyro_calib_cnt = gy.calib_cnt;
    out->gyro_calib_target = gy.calib_target;
    out->gyro_err = Chassis_Gyro_GetErrorCount();
    out->gyro_err_head = gy.err_head;
    out->gyro_err_flag = gy.err_flag;
    out->gyro_err_sum = gy.err_sum;
}
