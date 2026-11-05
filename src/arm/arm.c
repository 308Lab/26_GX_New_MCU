/**
  ******************************************************************************
  * @file    arm/arm.c
  * @brief   机械臂子系统入口：初始化编排、节拍、正式接口委托、调试数据汇总。
  ******************************************************************************
  */

#include "arm/arm.h"
#include "arm/arm_debug.h"
#include "arm/arm_config.h"
#include "arm/driver/step_uart.h"
#include "arm/driver/servo_bus.h"
#include "arm/driver/gripper.h"
#include "arm/driver/laser.h"
#include "arm/perception/joint_fb.h"
#include "arm/control/arm_actuator.h"
#include "arm/control/arm_kin.h"
#include "arm/control/arm_motion.h"
#include "arm/planning/arm_action.h"
#include "common/math_util.h"
#include "main.h"

#include <math.h>
#include <stddef.h>

static uint16_t s_servo_target[3];

static uint8_t servo_id_of(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return (uint8_t)ARM_SERVO_ID_J2;
    case ARM_AXIS_J3: return (uint8_t)ARM_SERVO_ID_J3;
    case ARM_AXIS_J4: return (uint8_t)ARM_SERVO_ID_J4;
    default: return 0U;
    }
}

static float servo_dir_of(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_DIR;
    case ARM_AXIS_J3: return ARM_SERVO_J3_DIR;
    case ARM_AXIS_J4: return ARM_SERVO_J4_DIR;
    default: return 1.0f;
    }
}

static float servo_pwm_per_deg_of(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_PWM_PER_DEG;
    case ARM_AXIS_J3: return ARM_SERVO_J3_PWM_PER_DEG;
    case ARM_AXIS_J4: return ARM_SERVO_J4_PWM_PER_DEG;
    default: return 2000.0f / ARM_SERVO_DEG_RANGE;
    }
}

static float servo_zero_pwm_of(uint8_t axis)
{
    switch (axis)
    {
    case ARM_AXIS_J2: return ARM_SERVO_J2_ZERO_PWM;
    case ARM_AXIS_J3: return ARM_SERVO_J3_ZERO_PWM;
    case ARM_AXIS_J4: return ARM_SERVO_J4_ZERO_PWM;
    default: return 1500.0f;
    }
}

/* 点动目标按关节软限位检查，而不是只钳 PWM */
static int servo_jog_limit_ok(uint8_t axis, int32_t pwm)
{
    float q = ((float)pwm - servo_zero_pwm_of(axis))
            / servo_pwm_per_deg_of(axis) * servo_dir_of(axis);

    switch (axis)
    {
    case ARM_AXIS_J2:
        return (q >= ARM_Q2_MIN_DEG && q <= ARM_Q2_MAX_DEG) ? 1 : 0;
    case ARM_AXIS_J3:
        return (q >= ARM_Q3_MIN_DEG && q <= ARM_Q3_MAX_DEG) ? 1 : 0;
    case ARM_AXIS_J4:
        return (q >= ARM_Q4_MIN_DEG && q <= ARM_Q4_MAX_DEG) ? 1 : 0;
    default:
        return 0;
    }
}

Arm_Status_e Arm_Init(void)
{
    Arm_Step_Init();
    Arm_Servo_Init();
    Arm_Gripper_Init();
    Arm_Laser_Init();
    Arm_Fb_Init();
    (void)Arm_Motion_Init();
    Arm_Actuator_Init();
    Arm_Act_Init();

    s_servo_target[0] = (uint16_t)ARM_SERVO_J2_ZERO_PWM;
    /* J3 零位 2813 超出 500~2500，S16 点动只能从 PWM 上限起步；
       点动前应先回读实际位置或重新标定。 */
    s_servo_target[1] = (uint16_t)ARM_SERVO_PWM_MAX;
    s_servo_target[2] = (uint16_t)ARM_SERVO_J4_ZERO_PWM;

    return ARM_OK;
}

void Arm_Update(void)
{
    /* 上电不主动发送任何总线命令；回读由按键或动作触发 */
    Arm_Step_Update();
    Arm_Servo_Update();
    Arm_Fb_Update();
    Arm_Motion_Update();
    Arm_Act_Update();
}

void Arm_Stop(void)
{
    Arm_Act_Abort();
    Arm_Motion_Abort();
    (void)Arm_Step_StopNow();
    Arm_Servo_StopAll();
}

uint8_t Arm_IsBusy(void)
{
    return (Arm_Act_IsBusy() != 0U || Arm_Motion_IsBusy() != 0U) ? 1U : 0U;
}

Arm_Status_e Arm_GetLastError(void)
{
    Arm_Status_e err = Arm_Act_GetError();

    if (err != ARM_OK)
    {
        return err;
    }

    if (Arm_Motion_HasFault() != 0U)
    {
        return Arm_Motion_GetError();
    }

    return ARM_OK;
}

Arm_Status_e Arm_Home(uint16_t timeout_ms)
{
    return Arm_Act_StartHome(timeout_ms);
}

Arm_Status_e Arm_Stow(uint16_t timeout_ms)
{
    return Arm_Act_StartStow(timeout_ms);
}

Arm_Status_e Arm_MoveJoint(const float q_deg[4], uint16_t timeout_ms)
{
    return Arm_Act_StartJoint(q_deg, timeout_ms);
}

Arm_Status_e Arm_MovePose(const Arm_Pose_t *pose, uint16_t timeout_ms)
{
    return Arm_Act_StartPose(pose, timeout_ms);
}

Arm_Status_e Arm_GetJoints(float q_deg[4])
{
    if (q_deg == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (Arm_Fb_GetJoints(q_deg) == 0)
    {
        return ARM_ERR_COMM;
    }

    return ARM_OK;
}

Arm_Status_e Arm_GetTcpPose(Arm_Pose_t *out)
{
    float q[4];

    if (out == NULL)
    {
        return ARM_ERR_PARAM;
    }

    if (Arm_GetJoints(q) != ARM_OK)
    {
        return ARM_ERR_COMM;
    }

    return (Arm_Kin_Forward(q, out) == 0) ? ARM_OK : ARM_ERR_PARAM;
}

Arm_Status_e Arm_NudgeTool(int16_t dx_mm, int16_t dy_mm, uint16_t timeout_ms)
{
    float q[4];
    float q1;
    float phi;
    Arm_Pose_t pose;
    float ox;
    float oy;
    float oz;

    if (Arm_GetJoints(q) != ARM_OK)
    {
        return ARM_ERR_COMM;
    }

    if (Arm_Kin_Forward(q, &pose) != 0)
    {
        return ARM_ERR_PARAM;
    }

    /* 协议 §2 工具系：x 左、y 上、z 指向目标（前） */
    q1 = q[0] * MATH_DEG2RAD;
    phi = (q[1] - q[2] + q[3]) * MATH_DEG2RAD;

    ox = (float)dx_mm * (-sinf(q1))
       + (float)dy_mm * (-sinf(phi) * cosf(q1));
    oy = (float)dx_mm * (cosf(q1))
       + (float)dy_mm * (-sinf(phi) * sinf(q1));
    oz = (float)dy_mm * cosf(phi);

    pose.x_mm += ox;
    pose.y_mm += oy;
    pose.z_mm += oz;

    return Arm_MovePose(&pose, timeout_ms);
}

Arm_Status_e Arm_ApproachTool(int16_t dist_mm, uint16_t timeout_ms)
{
    float q[4];
    float q1;
    float phi;
    Arm_Pose_t pose;

    if (Arm_GetJoints(q) != ARM_OK)
    {
        return ARM_ERR_COMM;
    }

    if (Arm_Kin_Forward(q, &pose) != 0)
    {
        return ARM_ERR_PARAM;
    }

    q1 = q[0] * MATH_DEG2RAD;
    phi = (q[1] - q[2] + q[3]) * MATH_DEG2RAD;

    pose.x_mm += (float)dist_mm * cosf(phi) * cosf(q1);
    pose.y_mm += (float)dist_mm * cosf(phi) * sinf(q1);
    pose.z_mm += (float)dist_mm * sinf(phi);

    return Arm_MovePose(&pose, timeout_ms);
}

Arm_Status_e Arm_PickAndPlace(const Arm_Pose_t *pick,
                              const Arm_Pose_t *place,
                              float approach_mm,
                              uint16_t timeout_ms)
{
    return Arm_Act_StartPickPlace(pick, place, approach_mm, timeout_ms);
}

Arm_Status_e Arm_Aim(const Arm_Pose_t *aim_pose, uint16_t timeout_ms)
{
    return Arm_Act_StartAim(aim_pose, timeout_ms);
}

Arm_Status_e Arm_Hold(void)
{
    return Arm_Act_Hold();
}

Arm_Status_e Arm_Release(void)
{
    return Arm_Act_Release();
}

Arm_Status_e Arm_Laser(uint8_t on)
{
    return (Arm_Actuator_Laser(on) == 0) ? ARM_OK : ARM_ERR_COMM;
}

/* ===================== 调试接口（只允许 app_debug 使用） ===================== */

Arm_Status_e Arm_Debug_StepJog(float delta_deg)
{
    int32_t pulses;

    pulses = (int32_t)lroundf(delta_deg * ARM_STEP_PULSES_PER_REV
                              * ARM_STEP_GEAR_RATIO / 360.0f);
    if (pulses == 0)
    {
        return ARM_ERR_PARAM;
    }

    if (Arm_Step_MoveRelativePulses(pulses, (uint16_t)ARM_STEP_SPEED_RPM) != 0)
    {
        return ARM_ERR_COMM;
    }

    Arm_Fb_AddJ1Delta(delta_deg);
    return ARM_OK;
}

Arm_Status_e Arm_Debug_StepStop(void)
{
    (void)Arm_Step_StopNow();
    return ARM_OK;
}

Arm_Status_e Arm_Debug_StepEnable(uint8_t on)
{
    return (Arm_Step_Enable(on) == 0) ? ARM_OK : ARM_ERR_COMM;
}

void Arm_Debug_StepRead(void)
{
    Arm_Step_RequestPosition();
}

Arm_Status_e Arm_Debug_ServoJog(uint8_t axis, float delta_deg)
{
    int32_t  target;
    int32_t  delta_pwm;
    uint8_t  slot;
    uint8_t  id;
    uint16_t pwm_now;

    if (axis < ARM_AXIS_J2 || axis > ARM_AXIS_J4)
    {
        return ARM_ERR_PARAM;
    }

    slot = (uint8_t)(axis - ARM_AXIS_J2);
    id = servo_id_of(axis);

    /* 必须从舵机实际位置起算：先取 PRAD 回读。
       没有回读时不动作，避免从零位假设产生大幅突跳。 */
    if (Arm_Servo_GetPositionPwm(id, &pwm_now) != 0)
    {
        Arm_Servo_RequestPosition(id);
        return ARM_ERR_COMM;
    }
    s_servo_target[slot] = pwm_now;

    delta_pwm = (int32_t)(delta_deg * servo_pwm_per_deg_of(axis)
                          * servo_dir_of(axis));

    target = (int32_t)pwm_now + delta_pwm;
    if (target < (int32_t)ARM_SERVO_PWM_MIN)
    {
        target = (int32_t)ARM_SERVO_PWM_MIN;
    }
    if (target > (int32_t)ARM_SERVO_PWM_MAX)
    {
        target = (int32_t)ARM_SERVO_PWM_MAX;
    }

    if (servo_jog_limit_ok(axis, target) == 0)
    {
        return ARM_ERR_LIMIT;
    }

    if (Arm_Servo_SetPwm(id, (uint16_t)target,
                         ARM_DEBUG_SERVO_MOVE_MS) != 0)
    {
        return ARM_ERR_COMM;
    }

    s_servo_target[slot] = (uint16_t)target;
    return ARM_OK;
}

void Arm_Debug_ServoRead(void)
{
    Arm_Fb_RequestAll();
}

Arm_Status_e Arm_Debug_Gripper(uint8_t close)
{
    return (Arm_Actuator_Gripper(close != 0U ? ARM_GRIPPER_CLOSE
                                            : ARM_GRIPPER_OPEN) == 0)
         ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Debug_GripperGrasp(void)
{
    return (Arm_Actuator_Gripper(ARM_GRIPPER_GRASP) == 0) ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Grasp(void)
{
    return (Arm_Actuator_Gripper(ARM_GRIPPER_GRASP) == 0) ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Close(void)
{
    return (Arm_Actuator_Gripper(ARM_GRIPPER_CLOSE) == 0) ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_MoveBaseRelative(float delta_deg)
{
    return Arm_Debug_StepJog(delta_deg);
}

Arm_Status_e Arm_SetWristPwm(uint16_t pwm, uint16_t time_ms)
{
    return (Arm_Servo_SetPwm((uint8_t)ARM_SERVO_ID_J4, pwm, time_ms) == 0)
         ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_MovePwm(const uint16_t pwm[3], uint16_t time_ms)
{
    const uint8_t ids[3] = {
        (uint8_t)ARM_SERVO_ID_J2,
        (uint8_t)ARM_SERVO_ID_J3,
        (uint8_t)ARM_SERVO_ID_J4
    };

    if (pwm == NULL)
    {
        return ARM_ERR_PARAM;
    }

    return (Arm_Servo_SetPwmSync(ids, pwm, 3U, time_ms) == 0)
         ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Debug_MovePwm(const uint16_t pwm[3], uint16_t time_ms)
{
    const uint8_t ids[3] = {
        (uint8_t)ARM_SERVO_ID_J2,
        (uint8_t)ARM_SERVO_ID_J3,
        (uint8_t)ARM_SERVO_ID_J4
    };

    if (pwm == NULL)
    {
        return ARM_ERR_PARAM;
    }

    return (Arm_Servo_SetPwmSync(ids, pwm, 3U, time_ms) == 0)
         ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Debug_ServoSetId(uint8_t old_id, uint8_t new_id)
{
    return (Arm_Servo_SetId(old_id, new_id) == 0) ? ARM_OK : ARM_ERR_COMM;
}

void Arm_Debug_ServoRequestId(uint8_t id)
{
    Arm_Servo_RequestId(id);
}

void Arm_Debug_ServoReadTarget(uint8_t id)
{
    Arm_Servo_RequestPosition(id);
}

int Arm_Debug_ServoGetPosition(uint8_t id, uint16_t *pwm)
{
    return Arm_Servo_GetPositionPwm(id, pwm);
}

void Arm_Debug_ServoClearCache(void)
{
    Arm_Servo_ClearPositionCache();
}

void Arm_Debug_ServoTorque(uint8_t id, uint8_t on)
{
    Arm_Servo_Torque(id, on);
}

void Arm_Debug_ServoTorqueBroadcast(uint8_t on)
{
    Arm_Servo_TorqueBroadcast(on);
}

uint8_t Arm_Debug_ServoGetTorque(uint8_t id)
{
    return Arm_Servo_GetTorque(id);
}

void Arm_Debug_ServoSetStart(uint8_t id)
{
    (void)Arm_Servo_SetStartPosition(id);
}

void Arm_Debug_ServoPreset12(void)
{
    const uint8_t  ids[2] = { 1U, 2U };
    const uint16_t pwm[2] = { 1823U, 1771U };

    (void)Arm_Servo_SetPwmSync(ids, pwm, 2U, ARM_DEBUG_SYNC_MOVE_MS);
}

void Arm_Debug_ServoQueryBroadcast(void)
{
    Arm_Servo_RequestBroadcastQuery();
}

void Arm_Debug_ServoScanStart(void)
{
    Arm_Servo_ScanStart((uint8_t)ARM_SERVO_SCAN_FIRST,
                        (uint8_t)ARM_SERVO_SCAN_LAST);
}

uint8_t Arm_Debug_ServoScanIsActive(void)
{
    return Arm_Servo_ScanIsActive();
}

uint8_t Arm_Debug_ServoScanIsDone(void)
{
    return Arm_Servo_ScanIsDone();
}

uint8_t Arm_Debug_ServoScanCount(void)
{
    return Arm_Servo_ScanCount();
}

uint8_t Arm_Debug_ServoScanId(uint8_t index)
{
    return Arm_Servo_ScanId(index);
}

Arm_Status_e Arm_Debug_Laser(uint8_t on)
{
    return (Arm_Actuator_Laser(on) == 0) ? ARM_OK : ARM_ERR_COMM;
}

Arm_Status_e Arm_Debug_Home(void)
{
    return Arm_Home(ARM_DEBUG_ACTION_TIMEOUT_MS);
}

Arm_Status_e Arm_Debug_Stow(void)
{
    return Arm_Stow(ARM_DEBUG_ACTION_TIMEOUT_MS);
}

Arm_Status_e Arm_Debug_MoveJoint(const float q_deg[4])
{
    return Arm_MoveJoint(q_deg, ARM_DEBUG_ACTION_TIMEOUT_MS);
}

Arm_Status_e Arm_Debug_MovePose(const Arm_Pose_t *pose)
{
    return Arm_MovePose(pose, ARM_DEBUG_ACTION_TIMEOUT_MS);
}

Arm_Status_e Arm_Debug_MoveTestJoint(void)
{
    const float q[4] = {
        ARM_TEST_Q1_DEG,
        ARM_TEST_Q2_DEG,
        ARM_TEST_Q3_DEG,
        ARM_TEST_Q4_DEG
    };

    return Arm_Debug_MoveJoint(q);
}

Arm_Status_e Arm_Debug_MoveTestPose(void)
{
    Arm_Pose_t pose;

    pose.x_mm = ARM_TEST_POSE_X_MM;
    pose.y_mm = ARM_TEST_POSE_Y_MM;
    pose.z_mm = ARM_TEST_POSE_Z_MM;
    pose.tool_deg = ARM_TEST_TOOL_DEG;

    return Arm_Debug_MovePose(&pose);
}

void Arm_Debug_GetView(Arm_Debug_View_t *out)
{
    uint8_t i;
    uint8_t fw;
    uint8_t hw;
    uint8_t last_id;

    if (out == NULL)
    {
        return;
    }

    (void)Arm_Fb_GetJoints(out->q_deg);

    out->servo_pwm[0] = 0U;
    out->servo_pwm[1] = 0U;
    out->servo_pwm[2] = 0U;
    out->step_fw = 0U;
    out->step_hw = 0U;

    for (i = 0U; i < ARM_AXIS_NUM; i++)
    {
        out->axis_online[i] = (uint8_t)Arm_Fb_IsOnline(i);
        out->err[i] = Arm_Fb_GetErrorCount(i);
    }

    for (i = ARM_AXIS_J2; i < ARM_AXIS_NUM; i++)
    {
        if (Arm_Fb_GetServoPwm(i, &out->servo_pwm[i - ARM_AXIS_J2]) != 0)
        {
            out->servo_pwm[i - ARM_AXIS_J2] = 0U;
        }
        out->servo_torque[i - ARM_AXIS_J2] =
            Arm_Servo_GetTorque(servo_id_of(i));
    }

    out->status = Arm_GetLastError();
    out->step_online = (uint8_t)Arm_Fb_IsOnline(ARM_AXIS_J1);
    out->laser_on = Arm_Actuator_LaserIsOn();
    out->laser_available = Arm_Laser_IsAvailable();
    out->gripper_closed =
        ((Arm_Actuator_GetGripper() == ARM_GRIPPER_CLOSE)
         || (Arm_Actuator_GetGripper() == ARM_GRIPPER_GRASP)) ? 1U : 0U;
    out->action = (uint8_t)Arm_Act_Current();
    out->motion_busy = Arm_IsBusy();
    out->step_err_kind = Arm_Step_GetLastErrorKind();
    out->step_last_ack = Arm_Step_GetLastAck();

    if (Arm_Servo_GetLastId(&last_id) == 0)
    {
        out->servo_last_id = last_id;
        out->servo_id_valid = 1U;
    }
    else
    {
        out->servo_last_id = 0U;
        out->servo_id_valid = 0U;
    }

    out->servo_id_result = Arm_Servo_GetIdResult();

    if (Arm_Step_GetVersion(&fw, &hw) == 0)
    {
        out->step_fw = fw;
        out->step_hw = hw;
    }
}
