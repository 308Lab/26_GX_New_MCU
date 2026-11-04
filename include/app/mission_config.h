/**
  ******************************************************************************
  * @file    app/mission_config.h
  * @brief   正式模式（含视觉上位机）任务层参数。
  *
  * 本文件只允许 .c 包含。
  ******************************************************************************
  */

#ifndef APP_MISSION_CONFIG_H
#define APP_MISSION_CONFIG_H

/* ===================== 1. 机械臂动作时序 ===================== */
#define MISSION_ARM_MS               (3000U)   /* PWM 位姿动作时间 */
#define MISSION_GAP_MS               (2000U)   /* 机械臂与步进/底盘之间的间隔 */
#define MISSION_GRIP_MS              (800U)    /* 夹爪动作等待 */
#define MISSION_J1_90_MS             (4000U)   /* J1 90° 约 3.5 s */
#define MISSION_J1_180_MS            (8000U)   /* J1 180° 约 7 s */
#define MISSION_LASER_MS             (3000U)   /* 激光最长 3 s */

/* ===================== 2. 视觉事务超时 ===================== */
#define MISSION_SCAN_CODE_TIMEOUT_MS (12000U)  /* 扫码 3 次事务 */
#define MISSION_SCAN_ORDER_MS        (4000U)   /* 扫描段 + 3 s 顺序兜底 */
#define MISSION_TRACK_TIMEOUT_MS     (5000U)   /* 单个追踪闭环 */
#define MISSION_TRACK_RETRY          (1U)      /* 追踪失败重粗调次数 */
#define MISSION_TRACK_CONV_SAMPLES   (3U)      /* 连续收敛样本数 */
#define MISSION_TRACK_TOL_MM         (5)       /* 收敛阈值 */
#define MISSION_SERVO_STEP_MAX_MM    (20)      /* C 底盘单步限幅 */
#define MISSION_SERVO_TOL_MM         (3)       /* C 伺服收敛阈值 */

/* ===================== 3. 任务阶段超时 ===================== */
#define MISSION_A_TRAVEL_MS          (40000U)
#define MISSION_B_TASK_MS            (30000U)
#define MISSION_C_TASK_MS            (25000U)
#define MISSION_D_TASK_MS            (30000U)
#define MISSION_RETURN_MS            (30000U)
#define MISSION_TOTAL_MS             (270000U)

/* ===================== 4. 底盘伺服 ===================== */
#define MISSION_C_SERVO_K            (1.0f)    /* dy -> 前后 mm，占位 */
#define MISSION_C_SERVO_STEP_MS      (2000U)

/* ===================== 5. 预存位（PWM，ID1/ID2/ID3） ===================== */
#define MISSION_POSE_COMMON_OPEN     { 1960U, 1502U,  868U }
#define MISSION_POSE_B_PRESET        { 1595U, 1504U, 1055U }
#define MISSION_POSE_B_BUCKET        { 1959U, 1642U, 1004U }
#define MISSION_POSE_C_PRESET        { 1427U, 1680U, 1914U }
#define MISSION_POSE_D_PRESET        { 1842U, 1642U, 1175U }
#define MISSION_POSE_STOW            { 2206U, 2114U, 2137U }

/* A 扫码姿态关节角（J2/J3/J4，J1 另给） */
#define MISSION_A_ARM_Q2_DEG         (150.44f)
#define MISSION_A_ARM_Q3_DEG         (164.28f)
#define MISSION_A_ARM_Q4_DEG         (36.45f)

#endif /* APP_MISSION_CONFIG_H */
