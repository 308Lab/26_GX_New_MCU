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
#define MISSION_A_SCAN_SETTLE_MS     (500U)    /* 到 A 点后先停稳，再发 0x10 */
#define MISSION_SCAN_CODE_TIMEOUT_MS (3000U)   /* 0x10 发出后 3 s 无响应判失败 */
#define MISSION_SCAN_ORDER_MS        (4000U)   /* 扫描段 + 3 s 顺序兜底 */
#define MISSION_ORDER_WAIT_MS        (3500U)   /* 停车后等视觉名次，含 3 s 兜底 */
#define MISSION_RANK_DIST_MM         (150)     /* 任务物品相对区域中间的前后距离 */
#define MISSION_MIDLINE_SPEED_MMPS   (50.0f)   /* 中位线对准的低速连续移动速度 */
/* v1.6：中位线回的是千分比偏移量，|offset| 小于死区即判「已对准」。
   50‰ = 5% 画宽，与上位机 v1.5.1 时代的三态判据（±5% 视为居中）同源，
   换成偏移量后判据不变，只是不再由视觉侧替 MCU 判。 */
#define MISSION_MIDLINE_DEADBAND_PERM (50)
#define MISSION_MIDLINE_PERIOD_MS    (500U)    /* 中位线请求周期 */
#define MISSION_MIDLINE_NO_RESP      (3U)      /* 连续无响应窗口数上限 */
#define MISSION_MIDLINE_TOTAL_MS     (5000U)   /* 对准总超时 */
#define MISSION_TRACK_TIMEOUT_MS     (5000U)   /* 单个追踪闭环 */
#define MISSION_TRACK_RETRY          (1U)      /* 追踪失败重粗调次数 */
#define MISSION_TRACK_CONV_SAMPLES   (3U)      /* 连续收敛样本数 */
#define MISSION_TRACK_TOL_MM         (5)       /* 收敛阈值 */
#define MISSION_SERVO_STEP_MAX_MM    (20)      /* C 底盘单步限幅 */
#define MISSION_SERVO_TOL_MM         (3)       /* C 伺服收敛阈值 */

/* ===================== 3. 任务阶段超时 ===================== */
#define MISSION_A_TRAVEL_MS          (40000U)
#define MISSION_B_TASK_MS            (90000U)  /* 展开+两次中位线+名次+取球放桶 */
#define MISSION_C_TASK_MS            (60000U)  /* 展开+两次中位线+名次+伺服+激光 */
#define MISSION_D_TASK_MS            (60000U)  /* 展开+两次中位线+名次+抓取 */
#define MISSION_RETURN_MS            (30000U)
#define MISSION_TOTAL_MS             (270000U)

/* ===================== 4. 底盘伺服 ===================== */
#define MISSION_C_SERVO_K            (1.0f)    /* dy -> 前后 mm，占位 */
#define MISSION_C_SERVO_STEP_MS      (2000U)

/* ===================== 5. 预存位（PWM，ID1/ID2/ID3） =====================
   审计 H2/H3 之后这些 PWM 要过 Arm_MovePwm 的软限位闸门：
   J4 868→920（原值 q4≈−93.3°，越过机械端点 897）；
   J3 2114/2127→2113（原值 q3=178.03°/180.0°，越过软限位 178°）。
   换算：q3=(PWM−937)/(595/90)，q4=(PWM−1559)/(2000/270)。 */
#define MISSION_POSE_A_SCAN          { 1976U, 2113U, 1829U }  /* A 扫码：小臂用初始值 */
#define MISSION_POSE_COMMON_OPEN     { 1960U, 1502U,  920U }
#define MISSION_POSE_B_PRESET        { 1595U, 1504U, 1055U }
#define MISSION_POSE_B_DEMO          { 1443U, 1642U, 1214U }  /* 追踪二次失败后的演示取球位 */
#define MISSION_POSE_B_BUCKET        { 1959U, 1642U, 1004U }
#define MISSION_POSE_B_BUCKET_DEMO   { 1756U, 1677U,  994U }  /* 追踪二次失败后的演示放桶位 */
#define MISSION_POSE_C_PRESET        { 1952U, 2027U, 1655U }  /* 2026-11-06 重标，腕部回调 −70 */
#define MISSION_POSE_D_PRESET        { 1842U, 1642U, 1175U }
#define MISSION_POSE_STOW            { 2206U, 2113U, 2137U }

/* A 扫码姿态关节角（J2/J3/J4，J1 另给） */
#define MISSION_A_ARM_Q2_DEG         (150.44f)
#define MISSION_A_ARM_Q3_DEG         (164.28f)
#define MISSION_A_ARM_Q4_DEG         (36.45f)

#endif /* APP_MISSION_CONFIG_H */
