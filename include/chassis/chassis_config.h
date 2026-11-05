/**
  ******************************************************************************
  * @file    chassis_config.h
  * @brief   底盘子系统唯一参数出口。
  *
  * 所有待实测参数集中于此。标 "TODO" 的必须在整定/标定阶段填写或修正。
  ******************************************************************************
  */

#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* ===================== 1. 机械与传动（TODO 实测） ===================== */
#define CHASSIS_WHEEL_DIA_MM        (65.0f)    /* 驱动轮直径 */
#define CHASSIS_TRACK_MM            (190.0f)   /* 左右履带中点距离 B（实测） */
#define CHASSIS_GEAR_RATIO          (30.0f)    /* 电机轴 -> 驱动轮 减速比 */
#define CHASSIS_ENC_PPR             (1024.0f)  /* 编码器线数 P/R（主理人确认） */

/* TIM_ENCODERMODE_TI1 = 2 倍频；若改为 TI12 必须同步改为 4 */
#define CHASSIS_ENC_COUNT_MULT      (2.0f)

/* 编码器输入滤波 0~15，0=不滤波。电机 PWM 噪声导致虚假计数时填 6~10 */
#define CHASSIS_ENC_FILTER_ICF      (0U)

/* 电机 A / 编码器 MA 是否为左轮。实测接线后调整 */
#define CHASSIS_MOTOR_A_IS_LEFT     (1)
#define CHASSIS_ENC_A_IS_LEFT       (1)

/* 安装极性补偿：1 = 逻辑正转对应物理反转
   S4 实测：左电机方向正确；右电机反；左右编码器计数方向均反 */
#define CHASSIS_MOTOR_L_INVERT      (0)
#define CHASSIS_MOTOR_R_INVERT      (1)
#define CHASSIS_ENC_L_INVERT        (1)
#define CHASSIS_ENC_R_INVERT        (1)

/* 派生量（勿手改） */
#define CHASSIS_PULSES_PER_REV  (CHASSIS_ENC_PPR * CHASSIS_ENC_COUNT_MULT * CHASSIS_GEAR_RATIO)

/* S5 标定：走固定脉冲量实际距离，得到 MM_PER_PULSE。
   标定完成后置 1，并把实测值填入下方。 */
#define CHASSIS_MM_PER_PULSE_USE_CAL  (1)

#if CHASSIS_MM_PER_PULSE_USE_CAL
/* 实测标定：目标 1000，DST 995，卷尺 998 */
#define CHASSIS_MM_PER_PULSE    (0.002242f)
#else
#define CHASSIS_MM_PER_PULSE    (3.14159265f * CHASSIS_WHEEL_DIA_MM / CHASSIS_PULSES_PER_REV)
#endif

#define CHASSIS_PULSE_PER_MM    (1.0f / CHASSIS_MM_PER_PULSE)

/* ===================== 2. 电机与 PWM ===================== */
#define CHASSIS_PWM_MAX             (1000)     /* TIM3 ARR = 999，占空比上限 */
/* 克服静摩擦的最小 PWM：左右分别标定，右轮机械阻力更大 */
#define CHASSIS_PWM_MIN_MOVE_L      (60)
#define CHASSIS_PWM_MIN_MOVE_R      (130)   /* 2026-11-01：110 低速抖动，折中 130 */

/* ===================== 3. 速度与加速度（TODO 实测） ===================== */
#define CHASSIS_V_CRUISE_MMPS       (300.0f)   /* 默认巡航线速度 */
#define CHASSIS_V_MIN_MMPS          (40.0f)    /* 低于此值按停车处理 */
#define CHASSIS_V_MAX_MMPS          (500.0f)
/* 起步加速度放缓，减小左右轮加速不同步造成的初始偏转 */
#define CHASSIS_A_ACCEL_MMPS2       (200.0f)
#define CHASSIS_A_DECEL_MMPS2       (500.0f)
/* 左右轮目标速度的变化率上限，用于平滑直道↔弯道的曲率突变。
   过高会削弱航向修正，过低会顿挫；1200 兼顾两者。 */
#define CHASSIS_WHEEL_ACCEL_MMPS2   (1200.0f)
#define CHASSIS_W_MAX_DPS           (90.0f)    /* 航向环修正上限（直行用） */

/* 原地旋转档位：1 = 高速档，0 = 常速档 */
#define CHASSIS_SPIN_PROFILE_FAST   (1)

#if CHASSIS_SPIN_PROFILE_FAST
#define CHASSIS_SPIN_W_MAX_DPS      (150.0f)   /* 高速原地旋转 */
#define CHASSIS_ALPHA_MAX_DPS2      (540.0f)
#else
#define CHASSIS_SPIN_W_MAX_DPS      (90.0f)    /* 常速原地旋转 */
#define CHASSIS_ALPHA_MAX_DPS2      (360.0f)
#endif

/* 原地旋转的末端处理 */
#define CHASSIS_SPIN_W_MIN          (40.0f)    /* 误差超死区时的最小角速度，保证越过右轮死区 */
#define CHASSIS_SPIN_DEAD_ZONE_DEG  (0.5f)     /* 与到点容差对齐：未到点就保持最小速度 */

/* ===================== 4. 节拍与滤波 ===================== */
#define CHASSIS_CTRL_PERIOD_MS      (5U)       /* CarTask */
#define CHASSIS_SENSE_PERIOD_MS     (5U)       /* EncTask */
#define CHASSIS_PATH_PERIOD_MS      (10U)      /* MainTask02 */
#define CHASSIS_DBG_PERIOD_MS       (200U)     /* OLEDTask */

/* 识别扫描段倒回中心时的速度（与扫描慢行速度独立） */
#define CHASSIS_SCAN_BACK_MMPS      (200.0f)

/* 速度一阶低通时间常数（ms），0 表示不滤波 */
#define CHASSIS_SPEED_LPF_TAU_MS    (20.0f)

/* 陀螺仪角度微分成角速度后的低通时间常数（ms） */
#define CHASSIS_GYRO_RATE_LPF_TAU_MS (20.0f)
#define CHASSIS_PID_DT_MIN_S        (0.001f)

/* ===================== 5. PID：航向角环（deg -> deg/s） ===================== */
/* 航向外环（角度 -> 目标角速度） */
#define CHASSIS_PID_H_KP            (7.00f)
#define CHASSIS_PID_H_KI            (2.50f)   /* 2026-11-01 回退：4.0 导致左右抖动 */
#define CHASSIS_PID_H_KD            (0.80f)   /* 微分阻尼：串级系统中仍不可省 */

/* 航向内环（角速度 -> 角速度指令），直接补偿打滑造成的转向损失 */
#define CHASSIS_PID_RATE_KP         (1.00f)
#define CHASSIS_PID_RATE_KI         (1.50f)
#define CHASSIS_PID_RATE_KD         (0.00f)
#define CHASSIS_PID_RATE_I_LIMIT    (45.0f)

/* 旧版前馈增益，保留以便回退 */
#define CHASSIS_HEADING_WZ_FF       (1.00f)
#define CHASSIS_PID_H_I_LIMIT       (90.0f)
#define CHASSIS_PID_H_D_ALPHA       (0.70f)
#define CHASSIS_PID_H_DEADBAND_DEG  (0.20f)  /* 与航向精度目标一致：0.2° 内不动作，避免抖动 */
#define CHASSIS_PID_H_I_SEP_DEG     (25.0f)

/* ===================== 6. PID：原地旋转角环（deg -> deg/s） ===================== */
#define CHASSIS_PID_S_KP            (3.00f)
#define CHASSIS_PID_S_KI            (0.20f)
#define CHASSIS_PID_S_KD            (0.30f)
#define CHASSIS_PID_S_I_LIMIT       (45.0f)
#define CHASSIS_PID_S_D_ALPHA       (0.70f)
#define CHASSIS_PID_S_DEADBAND_DEG  (0.10f)
#define CHASSIS_PID_S_I_SEP_DEG     (30.0f)

/* ===================== 7. PID：左右轮速度环（mm/s -> PWM） ===================== */
/* 左轮稳定；右轮机械间隙/摩擦不均，单独降低增益以抑制震荡 */
#define CHASSIS_PID_VL_KP           (2.00f)
#define CHASSIS_PID_VL_KI           (0.90f)
#define CHASSIS_PID_VL_KD           (0.00f)
#define CHASSIS_PID_VR_KP           (1.80f)
#define CHASSIS_PID_VR_KI           (0.70f)
#define CHASSIS_PID_VR_KD           (0.00f)
#define CHASSIS_PID_V_I_LIMIT       (250.0f)
#define CHASSIS_PID_V_D_ALPHA       (0.60f)
#define CHASSIS_PID_V_DEADBAND_MMPS (5.0f)
/* 禁用积分分离：速度环大误差时正需要积分快速提速，150 会导致高速上不去 */
#define CHASSIS_PID_V_I_SEP_MMPS    (0.0f)

/* 速度前馈：PWM_ff = Kff × 目标轮速(mm/s)
   由 S9 实测拟合：PWM ≈ KFF × 速度。左右分开以补偿机械差异。 */
#define CHASSIS_SPEED_FF_L          (1.42f)
#define CHASSIS_SPEED_FF_R          (1.35f)

/* ===== 圆弧补偿（B3）：标定层 + 径向比例反馈 =====
   S12 实测 R200 时实际半径约 135（-32.5%），根因是内侧履带低速打滑。
   V_SCALE/W_SCALE 来自实测比例，仅在 200 mm/s 工作点有效，需迭代标定。 */
/* 线速度补偿按转向方向分开：由实测两点线性反推 */
#define CHASSIS_ARC_V_SCALE_L       (1.23f)   /* 左转：实测 212 → 200 */
#define CHASSIS_ARC_V_SCALE_R       (1.49f)   /* 右转：实测 195 → 200 */
#define CHASSIS_ARC_W_SCALE         (0.93f)   /* 角速度补偿：57.3/61.9 */
#define CHASSIS_ARC_KR              (0.15f)   /* 径向比例反馈增益 deg/s per mm */
#define CHASSIS_ARC_V_MIN_MMPS      (120.0f)  /* 圆弧末端最低线速度，保证内轮不停 */

/* 预测式提前制动：检测延迟 + 制动响应 + 惯性，约 0.15 秒 */
#define CHASSIS_ARC_BRAKE_LEAD_S      (0.15f)
#define CHASSIS_ARC_BRAKE_LEAD_MAX_DEG (10.0f) /* 提前量上限，防止大速度时过早触发 */

/* ===================== 8. 到点判定与反向脉冲制动 ===================== */
#define CHASSIS_POS_TOL_MM          (5.0f)
#define CHASSIS_ANG_TOL_DEG         (0.2f)
/* 反向脉冲制动：按模式分开，各工况惯性不同 */
#define CHASSIS_BRAKE_PWM_MIN       (120)
#define CHASSIS_BRAKE_PWM_SCALE     (1.2f)    /* pwm = clamp(scale*|v|, min, max) */

/* 直行 */
#define CHASSIS_STRAIGHT_BRAKE_PWM_MAX    (400)
#define CHASSIS_STRAIGHT_BRAKE_PULSE_MS   (40U)

/* 圆弧：外侧轮速度高、惯性大 */
#define CHASSIS_ARC_BRAKE_PWM_MAX         (600)
#define CHASSIS_ARC_BRAKE_PULSE_MS        (60U)

/* 原地旋转 */
#define CHASSIS_SPIN_BRAKE_PWM_MAX        (400)
#define CHASSIS_SPIN_BRAKE_PULSE_MS       (40U)
#define CHASSIS_CORRECT_MAX_TIMES   (2U)
#define CHASSIS_ARRIVE_SETTLE_MS    (60U)     /* 到位后等待停稳再判完成 */

/* ===== 堵转保护（审计 H13）=====
   没有电流采样，用「目标轮速明显 + 实测几乎为零 + PWM 已饱和」判定卡死，
   持续 CHASSIS_STALL_HOLD_MS 后按故障结束本段（ctrl_finish(fault=1)）。
   目标门限取 60mm/s：低于它的目标在起步/末端收速度阶段天然「看着不动」。 */
#define CHASSIS_STALL_MIN_TARGET_MMPS (60.0f)
#define CHASSIS_STALL_SPEED_RATIO     (0.1f)  /* |v_meas| < 0.1·|v_target| */
#define CHASSIS_STALL_PWM_RATIO       (0.9f)  /* |pwm| >= 0.9·CHASSIS_PWM_MAX */
#define CHASSIS_STALL_HOLD_MS         (300U)  /* 连续成立多久才判故障 */

/* ===================== 9. 陀螺仪 JY61P ===================== */
#define CHASSIS_GYRO_UART_IRQ_PRIO  (6U)      /* 必须 >= 5，可被临界区屏蔽 */
#define CHASSIS_GYRO_TIMEOUT_MS     (200U)
#define CHASSIS_GYRO_BIAS_SAMPLES   (200U)    /* 零偏标定帧数，标定期间须静止 */
#define CHASSIS_GYRO_YAW_SIGN       (+1.0f)
#define CHASSIS_GYRO_YAW_SCALE      (1.000f)
#define CHASSIS_GYRO_WZ_SIGN        (+1.0f)

/* WIT 协议寄存器（TODO 以实物手册核对） */
#define CHASSIS_GYRO_REG_SAVE       (0x00U)
#define CHASSIS_GYRO_REG_RATE       (0x03U)
#define CHASSIS_GYRO_REG_BAUD       (0x04U)
#define CHASSIS_GYRO_REG_UNLOCK     (0x69U)
#define CHASSIS_GYRO_REG_YAW_ZERO   (0x76U)

/* 输出速率值：0x06=10Hz(默认) 0x07=20Hz 0x08=50Hz 0x09=100Hz 0x0B=200Hz
   10Hz 对 5ms 控制环太慢，启动时自动提到 100Hz */
#define CHASSIS_GYRO_RATE_VALUE     (0x09U)

/* 1 = 启动时自动配置输出速率（只写寄存器，不写 Flash 保存） */
#define CHASSIS_GYRO_AUTO_CONFIG    (1U)

/* ===================== 10. 里程计 ===================== */
/* 1.0 = 纯陀螺仪航向；<1 = 编码器/陀螺仪互补融合，抑制 JY61P 漂移 */
#define CHASSIS_ODOM_YAW_FUSE_ALPHA (0.98f)

/* ===================== 11. 规划层 ===================== */
#define CHASSIS_PLAN_MAX_SEGS       (32U)
#define CHASSIS_PLAN_TIMEOUT_FACTOR (3.0f)
#define CHASSIS_PLAN_SEG_GAP_MS     (0U)

/* 持续直行（ServoRun）的有限段超时余量（审计 H14）。
   名义段长 1000mm，理论时间 = 1000/|v| 秒，再加这个余量兜住加速与打滑。 */
#define CHASSIS_SERVO_TIMEOUT_MARGIN_MS (2000U)

/* 上电后是否自动执行 demo 路径。
   调试/标定阶段必须为 0，否则车体会自行运动。全部参数整定完再置 1。 */
#define CHASSIS_AUTO_RUN_PATH       (0U)

#endif /* CHASSIS_CONFIG_H */
