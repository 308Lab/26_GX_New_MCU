/**
  ******************************************************************************
  * @file    arm/arm_config.h
  * @brief   机械臂唯一参数出口。标 TODO 的项必须在实机标定后修正。
  *
  * 本文件只允许 .c 包含，不进入公共接口。
  ******************************************************************************
  */

#ifndef ARM_ARM_CONFIG_H
#define ARM_ARM_CONFIG_H

/* ===================== 1. ZDT X42S（Emm 固件，USART1） ===================== */
#define ARM_STEP_ADDR               (1U)
#define ARM_STEP_CHECK              (0x6BU)   /* 默认固定校验；若改为 XOR/CRC 需同步改驱动 */
#define ARM_STEP_PULSES_PER_REV     (3200.0f) /* TODO 以驱动器细分菜单为准（电机轴） */
#define ARM_STEP_GEAR_RATIO         (7.0f)    /* 小带大 7:1：输出轴 1 圈 = 电机 7 圈 */
#define ARM_STEP_SPEED_RPM          (30U)     /* TODO 按机械臂负载整定 */
#define ARM_STEP_ACCEL              (10U)     /* Emm 加速度档位 0~255 */
#define ARM_STEP_DIR_INVERT         (0)       /* 1 = 逻辑正方向与驱动方向相反 */
/* J1 无绝对零位：上电前手动对正，软件从 0° 开始累计相对移动。
   以下两项仅保留给原始回读诊断，不参与 J1 关节角计算。 */
#define ARM_STEP_ZERO_DEG           (0.0f)
#define ARM_STEP_DIR_SIGN           (+1.0f)
#define ARM_STEP_TIMEOUT_MS         (100U)
#define ARM_STEP_ONLINE_MS          (1000U)

/* ============ 2. 众灵 ZP45D / ZP10D 总线舵机（UART4，ASCII 协议） ============ */
#define ARM_SERVO_ID_J2             (1U)      /* TODO 逐个改 ID 后确认 */
#define ARM_SERVO_ID_J3             (2U)
#define ARM_SERVO_ID_J4             (3U)

#define ARM_SERVO_PWM_MIN           (500U)
#define ARM_SERVO_PWM_MAX           (2500U)
#define ARM_SERVO_DEG_RANGE         (270.0f)  /* 500~2500 对应 270° */

/* 每轴零位 PWM、方向与 PWM/度。
   J2/J4：2000/270 = 7.407 PWM/°。
   J3 于 2026-11-01 换新总线舵机，三点实测（主理人确认）：
     大小臂伸直（夹角 180°） -> PWM  895  (= q3   0°)
     小臂垂直大臂（夹角 90°）-> PWM 1605  (= q3  90°)
     收起/贴近大臂           -> PWM 2156  (= q3 ≈160°)
   斜率 = (1605-895)/90 = 7.8889 PWM/°；DIR=+1（q3 增大 → PWM 增大）；
   q3=0 的系统零位 = PWM 895。
   PWM 500 → q3≈−50°，PWM 2500 → q3≈203°，实际由软限位 0~160° 约束。 */
/* J2 于 2026-11-01 两点实测：向前伸直 PWM 846（q2=0），竖直向上 PWM 1522（q2=90），
   收起后仰 q2=181 → PWM 约 2205。斜率 = (1522-846)/90 = 7.5111 PWM/°；DIR=+1。 */
#define ARM_SERVO_J2_ZERO_PWM       (846.0f)
#define ARM_SERVO_J3_ZERO_PWM       (937.0f)   /* 2026-11-03 新 J3：90°=1532, 180°=2127 */
#define ARM_SERVO_J4_ZERO_PWM       (1559.0f)
#define ARM_SERVO_J2_DIR            (+1.0f)
#define ARM_SERVO_J3_DIR            (+1.0f)
#define ARM_SERVO_J4_DIR            (+1.0f)

#define ARM_SERVO_J2_PWM_PER_DEG    (676.0f / 90.0f)
#define ARM_SERVO_J3_PWM_PER_DEG    (595.0f / 90.0f)
#define ARM_SERVO_J4_PWM_PER_DEG    (2000.0f / ARM_SERVO_DEG_RANGE)

#define ARM_SERVO_TIMEOUT_MS        (50U)     /* PRAD / ID 应答超时 */
#define ARM_SERVO_ONLINE_MS         (1000U)

/* 地址扫描范围：逐 ID 发送 #IDPID!，避免多个舵机同时回复撞总线 */
#define ARM_SERVO_SCAN_FIRST        (0U)
#define ARM_SERVO_SCAN_LAST         (10U)
#define ARM_SERVO_SWITCH_DELAY_MS   (1U)      /* 收发方向切换后的稳定时间 */

/* 半双工方向脚极性。TODO 实机确认：TX 时 TX_EN 有效、RX_EN 无效，反之亦然 */
#define ARM_SERVO_TX_EN_LEVEL       (1U)
#define ARM_SERVO_RX_EN_LEVEL       (1U)

/* ===================== 3. 夹爪（TIM4 CH4，PD15，50 Hz） =====================
   按 270° PWM 舵机换算：PWM = 500 + 角度/270 × 2000
     关闭/初始 135° → 1500 us
     张开 45°     → 833 us
     夹取 105°    → 1278 us
   若夹爪舵机实际为 180°，需按 500 + 角度/180 × 2000 重算。 */
#define ARM_GRIPPER_OPEN_US         (833U)
#define ARM_GRIPPER_CLOSE_US        (1500U)
#define ARM_GRIPPER_GRASP_US        (1278U)
#define ARM_GRIPPER_MOVE_MS         (800U)

/* ===================== 4. 激光（GPIO） =====================
   2026-11-01 CubeMX 新增：PC1，标签 Laser。
   laser.c 语义：GPIO_PIN_SET = 开，GPIO_PIN_RESET = 关（高有效）。 */
#define ARM_LASER_ENABLE            (1)
#define ARM_LASER_GPIO_Port         Laser_GPIO_Port
#define ARM_LASER_Pin               Laser_Pin

/* ===================== 5. 几何与运动学（实测） =====================
   坐标：x 为车体正前，z 向上；θ1 绕 z 轴，pitch 面由 r/z 组成。
   H0：底座旋转中心到 θ2 轴的竖直距离。
   大臂：沿杆 120 mm；末端固定一段 20 mm、垂直大臂的偏置段，θ3 轴在偏置段末端。
         因此 θ2→θ3 轴距 = sqrt(120² + 20²) = 121.66 mm。
   小臂：θ3→θ4 160 mm，直连无折角。
   腕部 TCP：前 20 + 上 55 + 前 60 = 腕部局部坐标 (80, 55) mm。 */
#define ARM_BASE_H_MM               (55.0f)
#define ARM_UPPER_ALONG_MM          (120.0f)
#define ARM_ELBOW_OFFSET_MM         (20.0f)    /* 保留 20 mm 偏置 */
#define ARM_UPPER_AXIS_MM           (121.66f)  /* 参考：轴距 */
#define ARM_FOREARM_MM              (165.0f)   /* 2026-11-01 实测：小臂 165 */
#define ARM_TCP_FWD_MM              (80.0f)
#define ARM_TCP_UP_MM               (55.0f)

/* 关节偏置方向：+1 = 偏置朝 +n；实机方向相反时改 -1。 */
#define ARM_ELBOW_OFFSET_SIGN       (+1.0f)

/* 肘构型：-1 选择 q2 较大（大臂接近竖直/后仰）、小臂向前折的解。
   2026-11-01 按主理人直觉与实机姿态，改为 -1（原 +1 选出大臂前上 q2≈51° 的解）。 */
#define ARM_IK_ELBOW                (-1.0f)

/* ===================== 6. 关节软限位（实测） =====================
   J2/J3：前平伸/直臂到收拢姿态为机械极限；
   J4：向下 897 → -89.4°，收拢 2147 → +79.4°。
   软限位各留约 1° 余量。 */
/* J1 可 360° 连续旋转；软件角按 ±180° 环绕，运动取最短路径。 */
#define ARM_Q1_MIN_DEG              (-180.0f)
#define ARM_Q1_MAX_DEG              ( 180.0f)
#define ARM_Q2_MIN_DEG              (   0.0f)
#define ARM_Q2_MAX_DEG              ( 182.0f)
/* 2026-11-01 新 J3 舵机机械行程（q3=0° 伸直，q3≈160° 收起贴近大臂）：
   伸直 0° → PWM 895；收起 160° → PWM ≈2156。软限位直接取机械两端。 */
#define ARM_Q3_MIN_DEG              (   0.0f)
#define ARM_Q3_MAX_DEG              ( 178.0f)  /* 新 J3 机械上限 180°（PWM 2127），留 2° 余量 */
#define ARM_Q4_MIN_DEG              ( -89.0f)
#define ARM_Q4_MAX_DEG              (  79.0f)

/* 工作空间（以 J1 旋转中心为原点的水平半径 r / 高度 z）
   2026-10-30 临时放宽：为测试 (0,0,400) 竖直伸展位姿，
   r 下限放到 0、z 上限放到 420（r=0 物理最大 z≈415）。
   TODO：实机可达范围实测后按真值收紧。 */
#define ARM_R_MIN_MM                (0.0f)
#define ARM_R_MAX_MM                (400.0f)
#define ARM_Z_MIN_MM                (-100.0f)
#define ARM_Z_MAX_MM                (420.0f)

/* 收拢姿态：J2/J3 ≈ 180°，J4 ≈ +79°，各留约 1° 余量。
   J1 仍需实测。 */
#define ARM_STOW_Q1_DEG             (0.0f)
#define ARM_STOW_Q2_DEG             (181.0f)
#define ARM_STOW_Q3_DEG             (178.0f)  /* 收起贴近大臂：q3=178° → PWM≈2114 */
#define ARM_STOW_Q4_DEG             (78.0f)

/* ===================== 7. PTP 多轴同步 =====================
   2026-10-30 调试放慢：原 90°/s、0.2 s 太快。
   目标动作约 3 s：小角度由 MIN_TIME 兜底，大角度按 40°/s。
   TODO：整定完成后按比赛节拍调回。 */
#define ARM_PTP_SERVO_W_MAX_DEGPS   (40.0f)
#define ARM_PTP_STEP_W_MAX_DEGPS    (120.0f)
#define ARM_PTP_MIN_TIME_S          (3.00f)
#define ARM_PTP_POS_TOL_DEG         (5.0f)   /* 2026-11-01 调试放宽：2° 导致 ERR5，先放 5° */
#define ARM_PTP_TIMEOUT_MARGIN_MS   (1500U)
#define ARM_PTP_SETTLE_SAMPLES      (3U)
#define ARM_STEP_SPEED_MIN_RPM      (2U)
#define ARM_STEP_SPEED_MAX_RPM      (120U)

/* 审计 H11：`{G0000#...}` 同步帧既无长度也无 ACK，`HAL_UART_Transmit` 只能
   证明字节进了 FIFO。下发后在窗口内用 PRAD 回读位移确认帧真的被舵机接受。
   一轮四轴轮询约 200 ms，且要求看到约 1/4 的位移，所以窗口给 900 ms（报告
   写的 100 ms 跑不满一轮回读，做不到）。 */
#define ARM_PTP_VERIFY_WINDOW_MS    (900U)
#define ARM_PTP_VERIFY_MIN_PWM      (2U)     /* 目标与回读差 < 2 PWM（≈0.3°）视为无位移 */
#define ARM_PTP_VERIFY_MIN_MOVE_PWM (20U)    /* 总位移 < 20 PWM（≈2.7°）的微动不校验 */

/* 审计 H1：故障时发保持帧（目标=当前回读），50 ms 到达 */
#define ARM_PTP_HOLD_TIME_MS        (50U)

/* 默认测试位姿：S18 键 1 使用。
   (0,0,400) 竖直向上，tool_deg=91.09 使 q4≈0（腕部平伸、夹爪平行小臂）。
   数学解：q1=0, q2=51.23°, q3=39.85°, q4=0.00°；r=0 物理最大 z≈415。 */
#define ARM_TEST_POSE_X_MM          (0.0f)
#define ARM_TEST_POSE_Y_MM          (0.0f)
#define ARM_TEST_POSE_Z_MM          (400.0f)
#define ARM_TEST_TOOL_DEG           (91.09f)

/* PTP 联调测试关节角：S18 键 6 使用。
   q2=90（大臂竖直向上）、q3=0（小臂同向伸直），用于验证 J2/J3 标定。 */
#define ARM_TEST_Q1_DEG             (0.0f)
#define ARM_TEST_Q2_DEG             (90.0f)
#define ARM_TEST_Q3_DEG             (0.0f)
#define ARM_TEST_Q4_DEG             (0.0f)

/* ===================== 8. 反馈轮询 ===================== */
#define ARM_FB_POLL_MS              (200U)
#define ARM_FB_FAST_POLL_MS         (50U)     /* 按键强制回读时的间隔 */

/* ===================== 9. 调试点动 ===================== */
#define ARM_DEBUG_JOG_DEG           (10.0f)
#define ARM_DEBUG_SERVO_JOG_DEG     (5.0f)
#define ARM_DEBUG_SERVO_MOVE_MS     (3000U)  /* 点动 5° 用时：原 500 太快，2026-10-30 先放慢到 3 s，后续再调快 */
#define ARM_DEBUG_SYNC_MOVE_MS      (1000U)   /* 键 8 合并动作时间 */
#define ARM_DEBUG_ACTION_TIMEOUT_MS (8000U)

/* ===================== 10. 场地与物料高度（mm，离地） =====================
   主理人实测（2026-10-28）：
     地面到 J1 旋转中心约 30.40（已包含底板到旋转中心的高度）
     球座离地 15，球半径 20
     排爆桶离地 50，放置高度改离地 80
     人质台离地 100，单个人质高 70
   派生值以机械臂基座 z=0 为参考，供 mission 的 z 先验使用：
     B 球心      = 15 + 20 - 30.40 = 4.60
     B 放置点    = 80 - 30.40      = 49.60（按夹爪释放点理解，待确认）
     D 台面      = 100 - 30.40     = 69.60
     D 人质中心  = 100 + 35 - 30.40 = 104.60（待确认抓取部位）
     D 人质顶端  = 100 + 70 - 30.40 = 139.60
   TODO：桶/人质的水平 r 坐标、放置高度基准、抓取部位尚未提供。 */
#define ARM_BASE_HEIGHT_MM          (30.40f)  /* 地面到 J1 旋转中心（含底板偏移） */
#define ARM_BALL_SEAT_HEIGHT_MM     (15.0f)
#define ARM_BALL_RADIUS_MM          (20.0f)
#define ARM_BUCKET_HEIGHT_MM        (50.0f)
#define ARM_BUCKET_PLACE_HEIGHT_MM  (80.0f)
#define ARM_HOSTAGE_TABLE_HEIGHT_MM (100.0f)
#define ARM_HOSTAGE_HEIGHT_MM       (70.0f)

#define ARM_BALL_CENTER_Z_MM        (ARM_BALL_SEAT_HEIGHT_MM + ARM_BALL_RADIUS_MM - ARM_BASE_HEIGHT_MM)
#define ARM_BUCKET_PLACE_Z_MM       (ARM_BUCKET_PLACE_HEIGHT_MM - ARM_BASE_HEIGHT_MM)
#define ARM_HOSTAGE_TABLE_Z_MM      (ARM_HOSTAGE_TABLE_HEIGHT_MM - ARM_BASE_HEIGHT_MM)
#define ARM_HOSTAGE_CENTER_Z_MM     (ARM_HOSTAGE_TABLE_HEIGHT_MM + ARM_HOSTAGE_HEIGHT_MM * 0.5f - ARM_BASE_HEIGHT_MM)
#define ARM_HOSTAGE_TOP_Z_MM        (ARM_HOSTAGE_TABLE_HEIGHT_MM + ARM_HOSTAGE_HEIGHT_MM - ARM_BASE_HEIGHT_MM)

/* ===================== 11. 演示模式机械臂位姿（PWM） =====================
   数组顺序：ID1 / J2，ID2 / J3，ID3 / J4。2026-11-01 实测。
   演示模式 = 无视觉上位机。 */
/* 审计 H3：J4=868 → q4≈−93.3°，越过机械端点 897（−89.4°）。改 920 → q4≈−86.3°。 */
#define ARM_POSE_COMMON_OPEN_PWM    { 1960U, 1502U,  920U }  /* 通用左展开 */
#define ARM_POSE_B_PRESET_PWM       { 1595U, 1504U, 1055U }  /* B 机械臂预存 */
#define ARM_POSE_B_DEMO_PWM         { 1443U, 1642U, 1214U }  /* B 演示机械臂（取球） */
#define ARM_POSE_B_BUCKET_DEMO_PWM  { 1756U, 1677U,  994U }  /* B 演示放置桶 */
#define ARM_POSE_B_BUCKET_PRESET_PWM { 1959U, 1642U, 1004U } /* B 放置桶机械臂预存（正式） */
#define ARM_POSE_C_PRESET_PWM       { 1427U, 1680U, 1864U }  /* C 预存：腕部上移约 50 PWM */
#define ARM_POSE_D_PRESET_PWM       { 1842U, 1642U, 1175U }  /* D 机械臂预存 */
#define ARM_POSE_D_DEMO_PWM         { 1619U, 1573U, 1153U }  /* D 演示机械臂 */

#endif /* ARM_ARM_CONFIG_H */
