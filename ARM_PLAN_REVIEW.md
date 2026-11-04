# 机械臂方案核查：来源、原方案对照与分层检查

> 核查日期：2026-10-20
> 对照文档：`include/newPlan202610181940.md`（原调研方案）
> 核查对象：本轮已落地的机械臂驱动层、感知层与调试入口

---

## 1. 结论：哪些是参考的，哪些是自写的

| 部分 | 来源 | 说明 |
|---|---|---|
| ZDT X42S 协议 | **官方手册 V1.0.5** + Python 库 [emm_stepper](https://github.com/xhfb/emm_stepper) 参考 | 功能码、帧格式、校验、回读解析来自手册；**STM32 C 驱动为本工程自写** |
| 众灵 ZX361S 协议 | **众灵舵机手册** + Arduino 库 [serial-bus-servo](https://github.com/MagicCube/serial-bus-servo) + CSDN STM32 教程参考 | ASCII 指令、同步花括号、PRAD 回读来自手册/库；**C 驱动为本工程自写** |
| 串口分发、非阻塞节拍、调试页 | **本工程底盘既有风格** | 仿 `gyro.c` / `ctrl.c` 的单字节中断、static 状态、参数集中 |
| 四层目录与解耦规则 | **本工程底盘 + 原方案** | 按 `chassis` 的 driver/perception/control/planning 规划，当前只落地前两层 |
| 运动学、PTP、S 曲线、回读校正、示教 | **尚未实现** | 原方案建议留到后续阶段 |

**没有照搬任何整车开源项目**。SO-ARM/LeRobot、DevClaw 等的协议与硬件架构均与本项目不同，只作为方法论参考。

---

## 2. 原方案逐条对照

原方案的“最小可行算法栈”：

```
解析 IK → 关节空间 S 曲线同步插补 → 底座多轴时间对齐
→ 位置回读慢积分校正 → 回程差/齿隙补偿 → 重力下垂补偿表
→ 状态机 + 超时/报警 → 示教回放
```

| 原方案条目 | 当前状态 | 判断 |
|---|---|---|
| 执行器位置/速度环 | ZDT/舵机驱动器内部已闭环 | 符合，主控只回读 |
| 解析 IK（3R + 底座） | 已实现 `control/arm_kin.c` | 当前构型适用余弦定理解析解 |
| 关节空间 S 曲线 | 未实现；驱动器内部有加减速 | 第一版用“多轴时间对齐”替代，S 曲线按需 |
| 多轴时间对齐 | 已实现 `control/arm_motion.c` | 舵机同步帧 + 步进位置模式，同拍下发 |
| 位置回读 | 已实现（J1 `0x36`，J2~J4 `PRAD`） | 完成基础能力 |
| 回读慢积分校正 | 未实现 | 标定后加 |
| 回程差/齿隙补偿 | 未实现 | 舵机齿隙实测后加 |
| 重力下垂补偿 | 未实现（舵机无力矩接口） | 选做，用角度偏置表实现 |
| 状态机 + 超时/报警 | 动作/运动层有超时与 fault，统一安全 FSM 未做 | 下一步补入口安全状态机 |
| 示教回放 | 未实现 | 第三版可选 |
| 冗余分配/零空间 | 未实现 | **不适用**（实际 4 关节 + 夹爪） |
| DLS IK | 未实现 | 不需要，解析 IK 足够 |
| CTC/阻抗/动力学 | 不做 | 舵机无力矩接口，符合原方案“不建议照搬” |
| TOTG/TOPP-RA/MoveIt | 不做 | F103 资源不够，符合原方案 |

### 2.1 原方案与当前硬件的两处偏差

1. **底座不是 XY 两轴闭环步进**，而是 **J1 单轴步进**（ZDT X42S）。
   - 原方案“底座多轴时间对齐（两轴按距离比例缩放速度）”不适用。
   - 实际需要的是 **J1 步进与 J2/J3/J4 舵机跨总线的时间对齐**：同一控制拍下发，舵机用 `{G0000...}` 同步帧，步进用位置模式。
2. **自由度不是“5 DOF 对 4 DOF”**，而是 **4 关节（J1~J4）+ 夹爪 + 腕端激光**。
   - 冗余分配、零空间投影不需要。
   - J4 只做工具角补偿，不参与末端定位。

### 2.2 参考项目可信度核对

| 项目 | 调研结论 | 对本项目的用法 |
|---|---|---|
| [SO-ARM100/101 + LeRobot](https://github.com/huggingface/lerobot) | 官方仓库与文档完整，Feetech STS3215，提供同步读写、标定（homing_offset/range_min/max）、遥操作与回放 | 借鉴**标定流程与同步写思路**；协议不同，不能复用代码 |
| [DevClaw-Robot](https://github.com/sjkncs/DevClaw-Robot) | README 宣称 7 层算法、43 命令、12 状态 FSM；STM32F407 主站 + F103 从站 + CAN；完成度无独立验证 | 只参考 S 曲线/DLS/安全 FSM 的**分层思路**，不照搬 |
| [libscurve](https://codeberg.org/mikanet/libscurve) | 嵌入式 C，恒定 jerk S 曲线，5~31 µs/周期 | 若后续需要主控侧 S 曲线，可作为算法参考 |
| grbl/Marlin planner | 梯形/前瞻/DDA，成熟 | 步进底座若改为脉冲+方向时参考；当前驱动器自带规划 |
| SmartArm/Abot | F103 + CAN + 步进 + ROS MoveIt；1 kHz 插值 | 参考 F103 上的任务划分；本项目 5 ms 状态机足够 |
| 张大头 C 驱动 | **没有现成的 STM32 C 库**；官方/社区主要是 Python | 当前 C 驱动自写，按手册实现 |
| ZX361S 驱动 | Arduino 库最直接；CSDN 有 STM32 串口读取教程 | 当前 C 驱动自写 |

---

## 3. 全局与分层检查

### 3.1 依赖检查结果

| 检查项 | 结果 |
|---|---|
| `include/arm/*.h` 是否跨层 include | 通过：`arm.h`/`arm_debug.h` 只含 `stdint.h` + `arm_types.h`；各 driver/perception 头只含 `stdint.h` |
| `src/arm/driver/*.c` | 通过：只含自身、`arm_config.h`、`common/uart_rx.h`、`main.h` |
| `src/arm/perception/*.c` | 通过：只含自身、`arm_types.h`、`arm_config.h`、driver、`main.h` |
| `src/arm/arm.c` | 通过：含 arm 内部层，**无 chassis/vision** |
| `src/app/*.c` | 通过：只含 `arm/arm.h`；`app_debug.c` 额外含 `arm/arm_debug.h` |
| 跨层全局变量 | 通过：`extern` 只有 HAL 句柄 `huart1/huart4/htim4` |
| 同层互相依赖 | 通过：driver 之间互不调用；perception 只调 driver |

### 3.2 本轮修改的分层问题

| 问题 | 处理 |
|---|---|
| `step_uart` 原先接收角度并做角度→脉冲换算，驱动层承担了业务语义 | **已改**为 `Arm_Step_MoveRelativePulses()`；角度换算移到入口层（下一步移到 control） |
| `Arm_Debug_GetView` 原先直接读 `servo_bus` 的原始 PWM | **已改**为经 `Arm_Fb_GetServoPwm()`，入口只与 perception 交互 |
| ZX361S 同步帧缺少花括号 | **已修**为 `{G0000#...!}`（手册确认） |

### 3.3 仍存在的阶段性缺口（不是结构错误）

| 项 | 说明 | 计划 |
|---|---|---|
| `control/` 与 `planning/` 目录未创建 | 本轮只做驱动/感知，入口承担了少量控制职责 | 下一阶段创建，按原方案落地 IK/PTP/动作 |
| `Arm_Laser()` 直接调 driver | 尚无 `control/arm_actuator` | 下一阶段迁移 |
| `Arm_Debug_ServoJog()` 维护目标 PWM | 目标管理属于 control | 下一阶段迁移 |
| 正式动作接口（Home/Stow/Pick/Place/Aim）未声明 | 避免空实现/假接口 | 与 control/planning 一起落地 |
| 舵机零位、限位、齿隙未标定 | 参数未知 | 实机调试后写入 `arm_config.h` |
| 统一安全 FSM 未实现 | 目前只有驱动超时/错误计数 | control/入口层补 |

### 3.4 第二轮落地更新（2026-10-20）

本轮已把 `control/` 与 `planning/` 补齐：

| 模块 | 实现 | 说明 |
|---|---|---|
| `control/arm_kin` | 正解/逆解 | 3R 共面臂 + 底座旋转，J4 工具角补偿 |
| `control/arm_limit` | 关节软限位、工作空间 | 运动前统一检查 |
| `control/arm_actuator` | 夹爪/激光语义封装 | 唯一调用 gripper/laser 的 control 模块 |
| `control/arm_motion` | PTP 多轴时间对齐 | 舵机同步帧 + 步进位置模式，同拍下发；回读判定 + 超时 |
| `planning/arm_action` | Home/Stow/Joint/Pose/PickPlace/Aim | 非阻塞状态机，只调用 control |
| `arm.c` | 正式接口委托 | app 只看到 `arm.h` |

仍缺：

- 实机标定（连杆、零位、方向、限位、齿隙、收拢姿态）。
- 回读慢速微调（到位超差时单向趋近）。
- Pick&Place 实际坐标与夹爪时序。
- 统一安全 FSM（通信丢失、超时、急停锁存）。
- 示教回放、重力偏置表、主控侧 S 曲线。

### 3.5 调试旁路的边界

`arm_debug.h` 允许 `app_debug.c` 直接点动 J1/J2~J4、控制夹爪/激光，这是**临时调试旁路**：

- 只允许 `app_debug.c` 包含。
- 正式业务（mission/vision）只能使用 `arm.h`。
- 下一阶段 `control` 落地后，调试接口改为经 `control` 下发，入口层不再维护目标。

---

## 4. 下一步（按原方案顺序）

1. **control 层**：`arm_kin`（解析 IK）、`arm_limit`（软限位/工作空间）、`arm_actuator`（夹爪/激光语义封装）。
2. **arm_motion**：给定关节目标，计算基准时间 `T`；同一拍下发舵机同步帧 + 步进位置帧；回读判定到位。
3. **标定**：J1 脉冲/圈、舵机 PWM/角度、零位、方向、限位、齿隙；写入 `arm_config.h`。
4. **回读校正**：到位后比对，超差单向慢速微调。
5. **planning**：Home/Stow/Pick/Place/Aim 非阻塞动作序列；统一安全 FSM。
6. **mission/vision**：任务点编排；收臂后再行车。
7. 可选：示教回放、重力偏置表、S 曲线（libscurve 思路）。

---

## 5. 参考资料

- [ZDT X42S 手册 / emm_stepper](https://github.com/xhfb/emm_stepper)
- [Emm 协议说明（emm-stepper）](https://pypi.org/project/emm-stepper/)
- [y42_stepper 固件差异](https://github.com/xhfb/y42_stepper)
- [ZX361S Arduino 库](https://github.com/MagicCube/serial-bus-servo)
- [众灵科技舵机使用手册](https://www.renrendoc.com/paper/99607126.html)
- [ZX361 总线舵机使用说明书](https://www.renrendoc.com/paper/195891973.html)
- [SO-ARM / LeRobot](https://github.com/huggingface/lerobot)
- [LeRobot Feetech 同步读写与标定](https://mintlify.wiki/huggingface/lerobot/motors/feetech)
- [DevClaw-Robot](https://github.com/sjkncs/DevClaw-Robot)
- [libscurve](https://codeberg.org/mikanet/libscurve)
