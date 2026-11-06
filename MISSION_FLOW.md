# 26GX_New Mission 全流程（A/B/C/D）

> 最后更新：2026-10-27
> 状态：方案定稿，**未实现**。ArmTask 与单阶段底盘调试已落地；MissionTask、`arm_cmd`、mission 状态机尚未写代码。
> 依据：`AUDIT_REPORT.md`、`通信协议_视觉与MCU_v1.5.1_20261025.md`、现有路径/视觉/机械臂接口。
> 说明：本文是任务逻辑链与实现计划，不替代实车标定；所有未确认项集中在 §10。

## Context

赛程：起点 → A 读二维码 → B 排爆 → C 激光打靶 → D 救援搬运 → 返回区；全程自主、禁无线、整车 ≤250×250×300 mm、现场 ≤5 分钟。

现状（见 `AUDIT_REPORT.md`）：底盘、机械臂、视觉通信层各自可用，但**没有 mission**；路径事件仍由 debug 的 500 ms 假完成消费；视觉事件零消费者；机械臂正式动作零调用。本文给出完整任务逻辑链、状态机、事件契约和最小接口改动，作为后续实现依据。

已确认的业务规则：

1. **A**：扫码失败连续 3 次后放弃所有任务，连续跑完全图；配置类错误立即兜底。
2. **B**：现场有 3 个颜色不同的球（按前进方向有序）和 1 个桶；视觉端按二维码第一位颜色筛出目标球，0x2A 回目标球名次；先夹目标球，再放入唯一的桶（0x25/0x26/0x27）。
3. **C**：机械臂固定 Aim 位姿，不使用机械臂追踪微调；由底盘**前后运动、不转向**做视觉伺服，收敛后开激光。
4. **D**：按二维码第三位形状筛选，再用 0x49 顺序名次 0/1/2 选目标；只使用左侧展开，抓取后转正面带回返回区，**保持夹持不释放**。
5. **失败**：每点重试后跳过，继续后续，保证 5 分钟内跑完。

赛题 PDF 中文文本层损坏，无法提取细节；不能从原文确认的部分在 §10 列为待确认输入，并给出默认假设。

## 0. 任务架构（已定，2026-10-27）

| 任务 | 周期 | 优先级 | 栈 | 职责 |
|---|---:|---|---:|---|
| CarTask | 5 ms | Realtime | 512 B | `Chassis_Control_Update()` |
| EncTask | 5 ms | AboveNormal | 512 B | `Chassis_Sense_Update()` |
| MissionTask | 5 ms | Normal | 1 KB | `Vision_Link_Update()` + `Chassis_Path_Update()` + `Mission_Update()` |
| ArmTask | 5 ms | BelowNormal | 1 KB | `Arm_Update()` + 出队机械臂命令 |
| MainTask02 | 5 ms | Normal | 512 B | `Keypad_Scan()`、调试页、底盘校准、急停转发 |
| OLEDTask | 200 ms | Low | 512 B | OLED 刷新 |
| defaultTask | 1 s | Normal | 512 B | 空转 |

### 0.1 职责边界

- **MissionTask**：路径、视觉、mission 三者同任务，天然满足“视觉命令 API 与 `Vision_Link_Update()` 同上下文”和“路径事件单一消费者”两条契约。mission 的所有命令直接调用视觉/路径 API，无需加锁。
- **ArmTask**：机械臂状态机的唯一写者。所有正式动作、调试动作、停止/急停都通过静态命令队列 `arm_cmd` 入队，ArmTask 出队后调用 `Arm_*` 启动，并每 5 ms `Arm_Update()` 推进。
- **MainTask**：只扫键盘、刷新调试页、执行上电底盘校准；把 mission 启动/急停请求通过原子标志或 1 深队列转发。不再调用 `Chassis_Path_Update`、`Vision_Link_Update`、`Arm_Update`。
- **OLEDTask**：通过 `Mission_GetView`、`Chassis_Debug_GetView`、`Arm_Debug_GetView` 的只读快照显示，不直接读内部状态。
- **CarTask/EncTask**：保持现状，优先级最高，底盘控制与感知不受 mission/机械臂阻塞影响。

### 0.2 机械臂命令队列（`arm_cmd`）

```c
typedef enum {
    ARM_CMD_HOME, ARM_CMD_STOW, ARM_CMD_MOVE_JOINT, ARM_CMD_MOVE_POSE,
    ARM_CMD_PICK_PLACE, ARM_CMD_AIM, ARM_CMD_HOLD, ARM_CMD_RELEASE,
    ARM_CMD_LASER, ARM_CMD_NUDGE_TOOL, ARM_CMD_APPROACH_TOOL,
    ARM_CMD_STOP, ARM_CMD_DEBUG_KEY
} Arm_Cmd_Kind_e;

typedef struct {
    uint8_t     kind;
    uint8_t     key;          /* 调试命令用 */
    uint8_t     on;           /* 激光开关 */
    int16_t     a, b;         /* Nudge/Approach 参数 */
    Arm_Pose_t  pose;
    float       q[4];
    uint16_t    timeout_ms;
} Arm_Cmd_t;

void Arm_Cmd_InitRtos(void);                 /* App_InitRtos，静态队列 */
int  Arm_Cmd_Post(const Arm_Cmd_t *cmd);     /* 非阻塞，满返回 -1 */
uint8_t Arm_Cmd_Pop(Arm_Cmd_t *cmd);         /* ArmTask 每拍最多取 1 条 */
```

- 静态 `osMessageQueue`，深度 8，内存参照 `ctrl.c` 的 `attr.cb_mem/mq_mem` 静态分配。
- `Arm_Cmd_Post` 在 MissionTask/MainTask 调用；ArmTask 是唯一消费者。
- ArmTask 执行前检查 `Arm_IsBusy()`；BUSY 时保留当前命令在 ArmTask 内部重试（不重新入队），避免队列堆积。
- 完成/错误：ArmTask 维护 `volatile` 的 busy/error 快照；`Arm_IsBusy()`/`Arm_GetLastError()` 改为临界区快照读取，供 MissionTask 轮询。
- 调试命令 `ARM_CMD_DEBUG_KEY` 只在 `APP_STAGE_ARM` 使用；mission 构建不接收调试键。
- 急停 `ARM_CMD_STOP` 走同一队列；如担心队列满，保留一条旁路：MissionTask 置 `s_arm_estop` 原子标志，ArmTask 每拍优先检查并直接 `Arm_Stop()`。

### 0.3 MISSION 阶段启动与中止

- **上电自动启动**：MissionTask 等待 `App_ChassisReady()`（MainTask 完成 500 ms 静止 + 3 s 陀螺零偏标定后置位），再延时 `MISSION_AUTO_START_DELAY_MS`（默认 1000 ms，留出人员离开时间），自动 `Mission_Start()`。
- **中止**：MainTask 扫到按键 6 → `Mission_RequestAbort()`（原子标志）；MissionTask 在下一拍执行停路径、停追踪、`ARM_CMD_STOP`、关激光，进入 `MS_ABORT`。
- **复位**：`MS_ABORT`/`MS_ERROR` 下长按 6 或重新上电；复位后不自动重启，需重新上电，避免误动作。
- **OLED**：MISSION 阶段显示状态编号、当前区域、rank、追踪 dx/dy、耗时、最后错误；不显示机械臂调试页。

### 0.4 编译期阶段

- `APP_STAGE_CHASSIS`：保持当前底盘调试构建（MainTask 跑路径/视觉/调试，ArmTask 只跑 `Arm_Update`）。
- `APP_STAGE_ARM`：MainTask 只扫键/显示，调试键转 `ARM_CMD_DEBUG_KEY`；ArmTask 跑 `Arm_Update` + 调试命令；路径/视觉不初始化或空转。**在命令队列完成前不得启用此阶段。**
- `APP_STAGE_MISSION`：MissionTask 跑路径/视觉/mission；MainTask 只扫键/急停/校准；ArmTask 跑 `Arm_Update` + 正式命令。
- 比赛构建固定 `APP_STAGE_MISSION`，不包含 500 ms 假完成、不包含调试路径按键。

## 1. 逻辑链总览

```
IDLE
 └ Mission_Start
     └ PREP          检查陀螺/校准，Stow，清调试状态
         └ A_TRAVEL 表 to_A → ARRIVE('A')
             └ A_TASK  0x10 扫码 → Notify('A')
                 └ TO_B 表 A_B（扫描段）
                     └ ZONE_SCAN(B)  0x28/0x29（中位线暂不发）→ SCAN_READY → 倒回 → ARRIVE(B)
                         └ B_TASK  预设位 rank → 重新对中 → 0x22 微调 → 抓取 → 0x25 微调 → 放置 → Stow
                             └ TO_C 表 B_C
                                 └ C_TASK  0x38/0x39（中位线待定）→ 固定 Aim → 0x32 底盘伺服 → 0x34 → 激光 → 0x35/3s → 关
                                     └ TO_D 表 C_D
                                         └ D_TASK  0x48/0x49（中位线暂不发）→ 左展开 → 0x42 微调 → 抓取 → 左展开/转正面 → 带回（不释放）
                                             └ RETURN 表 D_end → 保持夹持 → DONE（不释放）
```

任何阶段可进入 `ERROR`（路径 fault、失联、机械臂 COMM、总超时）或 `ABORT`（人工急停）。

## 2. 状态机

### 2.1 顶层

| 状态 | 动作 | 等待 | 成功 | 失败/超时 |
|---|---|---|---|---|
| `MS_IDLE` | 激光关、停追踪、路径空闲 | `Mission_Start` | `MS_PREP` | — |
| `MS_PREP` | 检查陀螺在线/校准；`Arm_Stow` 或确认收拢；确保 mission 独占路径 | Stow 完成或 2 s | `MS_A_TRAVEL` | 陀螺故障/机械臂 COMM → `MS_ERROR` |
| `MS_A_TRAVEL` | 选 `to_A` 启动；同时下发**展开位姿** `g_arm_ready_pose`，不等待完成 | `ARRIVE('A')`；展开动作与行驶并行 | `MS_A_TASK` | `PATH_EV_ERROR`/40 s → `MS_ERROR`；展开失败只记录降级，不阻塞行驶 |
| `MS_A_TASK` | `RequestScanCode()` (0x10) | `SCAN_OK/ERROR/TIMEOUT` | 成功 → Notify('A') → `MS_TO_B` | 连续 3 次失败（超时/未检出/离线）→ 放弃任务，Notify('A') → `MS_DEGRADED`；配置类错误立即兜底 |
| `MS_DEGRADED` | 依次选 A_B/B_C/C_D/D_end；不调用视觉命令、不追踪、机械臂保持 Stow | `SCAN_READY`/`ARRIVE`/`DONE` | ARRIVE 当拍 Notify，D_end 完成 → `MS_DONE` | 总超时 → `MS_ERROR` |
| `MS_TO_B` | 选 `A_B` 启动 | `SCAN_BEGIN('B')` | `MS_ZONE_SCAN` | `PATH_EV_ERROR`/45 s → `MS_ERROR` |
| `MS_ZONE_SCAN` | 共用扫描流程（见 2.2） | `SCAN_READY`、`ORDER` | 按区进入 B/C/D 任务 | 降级后仍进入任务 |
| `MS_B_TASK` / `MS_C_TASK` / `MS_D_TASK` | 见 2.3-2.5 | 各自 | 下一段行驶 | 重试后跳过 |
| `MS_RETURN` | 选 `D_end` 启动，保持夹持 | `PATH_EV_DONE` | 保持夹持 → `MS_DONE`（不释放） | 30 s → `MS_ERROR` |
| `MS_DONE` | 关激光；D 正常完成保持夹持不 Stow；降级/跳过场景 Stow；记录耗时 | 人工取走/复位 | `MS_IDLE` | — |
| `MS_ERROR` / `MS_ABORT` | 停路径、停臂、关激光、停追踪 | 复位键 | `MS_IDLE` | — |

### 2.2 B/C/D 共用扫描流程

> **2026-11-05 变更**：已取消慢行扫描段与路径层按 rank 倒回。现流程为：路径停车于任务区中间 → 展开机械臂 → 请求视觉名次 → 按 rank 用 `ServoStep` 前后移动（±150 mm）。以下表格为历史规划，最新实现见 §12.3~§12.5。

| 子状态 | 动作 | 等待 | 成功 | 失败/超时 |
|---|---|---|---|---|
| `ZONE_SCAN` | 收到 `PATH_EV_SCAN_BEGIN(zone)` 后 `StartScan(zone)`；**B 点不发中位线**，直接 `RequestOrder(zone)`；C/D 待定 | `ORDER`、`SCAN_READY`（若启用中位线则加 `MIDLINE`） | `SCAN_READY` → 等倒回 | 8 s 降级 rank=1 |
| 中位线处理（暂不使用） | B 点不调用 0x20；接口保留给 C/D 评估。若后续启用：value=1 → `RequestOrder`；2/3 → 不修正；0 → 重发 | — | — | — |
| 顺序处理 | `ORDER` 事件 value 0/1/2；`flags bit0` 表示 3 s 兜底 | — | `s_rank=value` | 3.5 s → rank=1 |
| `ZONE_ARRIVE` | 收到 `ARRIVE(zone)`：**当拍立即 `Notify(zone)`**，取消未决中位线 | `ORDER`（≤3.5 s） | `Chassis_Path_StartPreset(zone, rank)` | 无 order → rank=1 |
| `ZONE_PRESET` | 按 rank 去目标预设位 | `PATH_EV_DONE` | `ZONE_REALIGN` | 12 s → 跳 realign |
| `ZONE_REALIGN` | **B 点跳过**；C/D 待定。若启用：再次 `RequestMidline(zone)` | `MIDLINE` | 任意值 → 区域任务 | value=0 → 重发；5 s 降级 |

**完成契约**：B/C/D 的路径表以扫描段结尾；`ARRIVE` 当拍 `Notify` 后路径完成并发 `DONE`，mission 在路径空闲状态下执行任务，彻底绕开 `SEG_TASK_WAIT` 无超时。

### 2.3 B 点排爆

| 状态 | 动作 | 等待 | 成功 | 失败 |
|---|---|---|---|---|
| `B_PICK_ABOVE` | `Arm_MovePose(g_b_pick_above[rank])` | 动作完成 | `B_TRACK_OBJECT` | BUSY 重试 1 s；其他错误跳过 B |
| `B_TRACK_OBJECT` | `TrackStart(B_OBJECT, 50)` (0x22) | 每 100 ms `GetTrack()`；`Arm_IsBusy()` 时不叠加 | 连续 3 样本 ≤5 mm → `TrackStop` (0x24) + `GetTcpPose` | 5 s/5X → 重粗调 1 次 → 仍失败跳过 B |
| `B_DESCEND` | `Arm_ApproachTool(+30 mm)` | 动作完成 | `B_GRASP` | 错误 → 回观察位、跳过 |
| `B_GRASP` | `Arm_Hold()`；等 600~800 ms | 定时 | `B_RETRACT` | — |
| `B_RETRACT` | `Arm_ApproachTool(-30 mm)` | 动作完成 | `B_PLACE_ABOVE` | 错误 → 跳过 |
| `B_PLACE_ABOVE` | `Arm_MovePose(g_b_place_above)`（固定观察位，不按 rank） | 动作完成 | `B_TRACK_BUCKET` | 错误 → 跳过 |
| `B_TRACK_BUCKET` | `TrackStart(B_BUCKET, 50)` (0x25) | 同物体追踪 | 收敛 → `TrackStop` (0x27) | 5 s/5X → 重粗调 1 次 → 仍失败跳过放置 |
| `B_PLACE_DESCEND` | `Arm_ApproachTool(+20 mm)` | 动作完成 | `B_RELEASE` | 错误 → 原地释放 |
| `B_RELEASE` | `Arm_Release()`；等 600 ms；`ApproachTool(-20)` | 动作完成 | `B_READY` | — |
| `B_READY` | `Arm_MovePose(g_arm_ready_pose)` 回展开位姿 | 动作完成或 3 s | 进入 C | 记录降级继续 |

#### 2.3.1 执行顺序（正常路径，B 点暂不发中位线）

**阶段 1：扫描与定位（底盘运动中）**

1. 表 A_B 首段 `PATH_CALIB` 停车重标陀螺零偏，约 5 s。
2. 直行 1350 → 右弧 90° → 直行 385 → 右弧 90°，进入扫描段。
3. 路径层发 `PATH_EV_SCAN_BEGIN('B')`。
4. MissionTask 收到后：`StartScan(B)` 0x28；**不发 0x20**；随即 `RequestOrder(B)` 0x29（车前进中视觉按二维码第一位颜色筛球并算名次）。
5. 扫描段慢行 700 mm @100；等待 0x2A（视觉建议 3 s 内回）。
6. 扫描段走完发 `PATH_EV_SCAN_READY('B')`；若还没收到 ORDER，继续等（最多 3.5 s）。
7. 路径停 300 ms → 倒回 350 mm @200 → 停稳 → `ARRIVE('B')`。
8. mission 收到 ARRIVE：**当拍 `Notify('B')`**，路径完成、底盘空闲；`rank = ORDER.value`，无 ORDER → `rank=1`。

**阶段 2：左侧展开中转 → 预存球位 → 追踪球**

9. `Arm_MovePose(g_b_left_ready)`：机械臂进入**左侧展开态**作为中转（空载）。
10. `Arm_MovePose(g_b_pick_above[rank])`：到对应目标球的预存位。
11. `TrackStart(B_OBJECT, 50)` 0x22。
12. 每 100 ms：`GetTrack(B_OBJECT)`；`Arm_IsBusy()` 时跳过本拍；否则 `Arm_NudgeTool(dx, dy, 2000)`（死区 2 mm，限幅 ±80 mm）。
13. 连续 3 个新鲜样本 `|dx|,|dy| ≤ 5 mm` → `TrackStop(B_OBJECT)` 0x24；`GetTcpPose`。
14. 5 s 未收敛或 `TRACK_LOST_5X` → 回 `g_b_left_ready` → 重新 0x22 一次；仍失败跳过 B。
15. `Arm_ApproachTool(+30 mm)` → `Arm_Hold()` → 等 600~800 ms → `Arm_ApproachTool(-30 mm)`。

**阶段 3：持球左展开 → 右侧展开 → 预存桶位 → 追踪桶 → 放置**

16. `Arm_MovePose(g_b_left_ready_carry)`：夹着球回到左侧展开（持球变体）。
17. `Arm_MovePose(g_b_right_ready_carry)`：换右侧展开（持球变体）。
18. `Arm_MovePose(g_b_bucket_above)`：到唯一桶的预存位。
19. `TrackStart(B_BUCKET, 50)` 0x25。
20. 闭环同 12；连续 3 样本 ≤5 mm → `TrackStop(B_BUCKET)` 0x27；`GetTcpPose`。
21. 5 s/5X → 回 `g_b_right_ready_carry` → 重新 0x25 一次；仍失败用名义桶位直接放置。
22. `Arm_ApproachTool(+20 mm)` → `Arm_Release()` → 等 600 ms → `Arm_ApproachTool(-20 mm)`。

**阶段 4：右侧展开中转 → 收起 → 前往 C**

23. `Arm_MovePose(g_b_right_ready)`：空载右侧展开，作为动作中转。
24. `Arm_Stow(3000)`：收起机械臂（去 C 的行驶姿态）。
25. 确认 0x24/0x27 已发、无 pending 视觉事务、激光关。
26. 选 `B_C` 表启动，前往 C。

#### 2.3.2 业务约束与分支

- B 点现场：**3 个颜色不同的球**，按前进方向排列；**桶只有一个**。
- 视觉端在 A 点保存二维码第一位颜色；收到 0x29 后按该颜色筛球，0x2A 返回目标球名次（0=最近/1=中/2=最远）。MCU 只按 rank 去预存位，不感知颜色。
- **只夹二维码第一位颜色对应的那一个球**，另外两个不处理；顺序固定为**先夹球、再放桶**。
- **中位线暂不发**：B 点不调用 0x20/0x21；底盘定位完全靠路径表停车位置 + 预存位。接口保留给 C/D 评估。
- **左侧/右侧展开**是机械臂动作中转，不是任务位；空载与持球可能用不同位姿（`*_carry`），需分别标定。
- 0x25 直接追踪唯一的桶；`g_b_bucket_above` 是桶的预存位。
- **重粗调定义**：停追踪 → 回对应展开/观察位 → 重新 `TrackStart`；5 s 仍不收敛则跳过。
- 0x28/0x29 发送失败：100 ms 重试，最多 10 次；仍失败 → 跳过 B 的视觉任务，直接行驶（Notify 已在 ARRIVE 发出）。
- 顺序 3 s 兜底：rank=1；ARRIVE 后再等 3.5 s。
- 目标颜色球未检出：协议只允许 0x2A=1；MCU 会去中位预存位，可能抓错；视觉端应尽量用 `0x12(0x04)`。
- 机械臂 BUSY：50 ms×20；TIMEOUT：Stop 后重试 1 次；COMM：J1 离线则中止整链，否则跳过 B。
- 夹爪无回读：Hold/Release 后按 600~800 ms 时间等待。
- B 任务总超时：从 ARRIVE 起 30 s；超时 → 停追踪、回展开态、跳过 B。
- 跳过 B/离开 B 前必须确保 0x24/0x27 已发、夹爪状态安全（已夹球则先放置或保持夹持）、机械臂完成右侧展开中转后 Stow。

#### 2.3.3 协议数据支撑与缺口

**协议提供且可用于 B 流程的数据**

| 数据 | 命令 | 含义 | 用于 |
|---|---|---|---|
| 扫码成功/错误 | 0x11 / 0x12 | 0x11 无载荷，三位数仅视觉内部用 | A 点判定 |
| 顺序名次 | 0x29 → 0x2A | 目标球在前进方向升序名次 0/1/2 | 选预存球位 |
| 球追踪偏移 | 0x22 → 0x23 | 工具系平面 dx/dy（4B），原点=夹爪 TCP，x 左 y 上，无 z | 机械臂微调 |
| 桶追踪偏移 | 0x25 → 0x26 | 同上 | 机械臂微调 |
| 停止追踪 | 0x24 / 0x27 | 视觉停流 | 结束闭环 |
| 丢失/5X | 0x13 / 事件 | 失稳或连续丢失 | 重粗调 |
| TCP 位姿 | 0x14 | MCU→视觉，视觉只记录 | 不能作为闭环反馈 |
| 心跳 | 0xFE | 视觉→MCU 5Hz；500 ms 失联 | 链路状态 |

**协议不能提供、流程必须自己兜底的部分**

1. **无 z/深度**：协议明确只给平面 dx/dy；法向距离、夹取高度、放置高度全开环，靠预存位和 `ApproachTool`。
   - **可用先验高度补 z**（符合协议 §1“z 由 MCU 按预设距离/接近策略处理”）：
     - B 点夹爪/腕部相机朝下时，工具 z 竖直向下，`dx/dy` 是水平面；球心相对机械臂底盘的高度、桶上方放置点相对底盘的高度可作为 z 先验。
     - `target_z = 标定高度`；`approach_dist = TCP_z - target_z`，用 `Arm_ApproachTool` 沿工具法向接近。
     - 需要标定：机械臂基座相对台面高度、球半径/球心高度、桶沿高度、释放点高度。
     - 前提：工具 z 竖直、台面/球/桶高度固定；高度误差会直接变成抓取/放置误差，台面变化需重标。
     - 若工具 z 不竖直，必须用 FK 的 `R_tool→base` 转换，不能直接把高度当 z。
2. **无目标 ID**：0x22 不指定追哪个球；视觉必须按 A 点二维码第一位颜色自动筛球。0x25 只有唯一桶，不受影响。
3. **无绝对坐标**：只有相对 dx/dy；预存位必须准确，小偏差可修，大偏差不可修。
4. **无夹取/放置确认**：夹爪 PWM 无回读，协议无回执；只能按时间，不能确认是否夹住。
5. **追踪 50 Hz 建议 vs PTP 最小 200 ms**：数据频率够，执行器只能做约 3~5 Hz 闭环。
6. **B 点不发中位线**：底盘停车偏差直接转为机械臂预存位偏差，没有底盘对准补偿。
7. **MCU→视觉命令无送达确认**：0x28/0x22/0x25/0x24/0x27 丢帧只能重试或时序兜底。
8. **dx/dy 轴向与符号**：协议定义 x 左、y 上、z 法向；相机安装偏差和镜像必须现场验证，否则微调方向相反。

**结论**：协议能支撑“预存位粗定位 + 平面 dx/dy 微调 + 开环接近/夹取/放置”的流程，**不能支撑全闭环**。要全程走完，必须标定：左/右展开（空载/持球）、三个球预存位、桶预存位、`tool_deg`、接近距离、夹爪行程；并确认视觉端能按二维码颜色选球。

### 2.4 C 点激光打靶（机械臂固定 Aim + 底盘只前后伺服）

| 状态 | 动作 | 等待 | 成功 | 失败 |
|---|---|---|---|---|
| `C_PRESET` | `Chassis_Path_StartPreset('C', rank)` | `PATH_EV_DONE` | `C_AIM` | 12 s → 原地进入 Aim |
| `C_AIM` | `Arm_MovePose(g_c_aim)` 固定 Aim；完成后强制 `Arm_Laser(0)` | 动作完成 | `C_TRACK` | 错误 → 跳过 C |
| `C_TRACK` | `TrackStart(C, 50)` (0x32)；每 200 ms 读 `GetTrack(C)` | 连续 3 样本 `|dy| ≤ 3 mm`（`dx` 只记录、不动作） | `TrackStop` (0x34) → `C_FIRE` | 4 s 未收敛 → 用最后位置停追踪 |
| 伺服单步 | `Chassis_Path_ServoStep(forward, 0)`；`forward` 由 `dy` 决定 | `PATH_EV_AUX_DONE` | 继续读取 | 2 s/步 → 跳过本步 |
| `C_FIRE` | `Arm_Laser(1)`，记录 `fire_start_ms` | `C_TARGET_DOWN`（tick ≥ 起点）或 3 s | `Arm_Laser(0)` → `C_LASER_OFF` | 3 s 超时关激光、记降级 |
| `C_LASER_OFF` | `Arm_Laser(0)`；`Arm_Stow(3000)` | 动作完成 | 进入 D | 记录降级继续 |

#### 2.4.1 执行顺序

**阶段 1：扫描与选靶（底盘运动中）**

1. B→C 表直行 1400，进入扫描段。
2. `PATH_EV_SCAN_BEGIN('C')` → `StartScan(C)` 0x38；**不发 0x30**；随即 `RequestOrder(C)` 0x39。
3. 扫描慢行 700 @100；视觉按**二维码第二位颜色**筛靶，`0x3A` 返回名次 0/1/2。
4. `SCAN_READY` → 停 300 ms → 倒回 350 → `ARRIVE('C')`。
5. ARRIVE 当拍 `Notify('C')`；`rank = 0x3A`，无则 1。

**阶段 2：到靶预存位**

6. `Chassis_Path_StartPreset('C', rank)`：去对应靶的预存位（三张表，距离待实测）。
7. 等 `PATH_EV_DONE`；12 s → 原地继续。

**阶段 3：固定 Aim**

8. `Arm_MovePose(g_c_aim)`：**固定 Aim 位姿，不随 rank 改变**；rank 只选底盘预存位。
9. 完成后 `Arm_Laser(0)` 兜底，防止提前照射；同时修正 `arm_action.c` 里 Aim 完成自动开激光的行为。

**阶段 4：底盘前后伺服 + 追踪**

10. `TrackStart(C, 50)` 0x32。
11. 每 200 ms：`GetTrack(C)`；只取 `dy` 作为前后偏差；`dx` 只记录、不动作。
12. `forward = -dy * K_move`，限幅 ±20 mm/步；`Chassis_Path_ServoStep(forward, 0)`；等 `PATH_EV_AUX_DONE` 再读下一帧。
13. 连续 3 个新鲜样本 `|dy| ≤ 3 mm` → `TrackStop(C)` 0x34；4 s 未收敛 → 用最后位置停追踪。
14. 伺服期间机械臂保持 Aim，不再下发机械臂动作。

**阶段 5：激光**

15. `Arm_Laser(1)`，记录 `fire_start_ms`。
16. 等 `C_TARGET_DOWN`（tick ≥ `fire_start_ms`）或 3 s；无论哪种 → `Arm_Laser(0)`。
17. `Arm_LaserIsAvailable()==0` → 记录降级，跳过 15-16。

**阶段 6：离开 C**

18. 确认 0x34 已发、激光已关。
19. `Arm_Stow(3000)` 收起（去 D 的行驶姿态）。
20. 选 `C_D` 表启动，前往 D。

#### 2.4.2 约束与数据缺口

- 靶：按二维码第二位颜色筛；三个靶的预存位需实测。
- **只前后、不转向**：`dx` 无法修正，只记录；左右对准靠路径停车位置 + 靶预存位。
- **Aim 固定**：rank 不改变机械臂 Aim；只有底盘预存位随 rank 变。
- 中位线暂不发；底盘停车偏差直接转成靶预存位偏差。
- 追踪 5 s/5X → 重粗调一次：底盘回靶预存位 → 重新 0x32；仍失败跳过 C。
- 底盘伺服超时/路径 fault → 跳过 C。
- 激光出射点与 TCP 的偏移需标定；0x34 后开激光，0x35 或 3 s 关。
- 数据缺口：协议无 z；C 只做平面瞄准，前后由 `dy` 驱动，激光聚焦距离靠预存位与前后伺服。
- `dx` 只记录不动作，现场必须确认靶预存位左右精度；若左右偏差大，需要重新启用转向伺服。

### 2.5 D 点救援搬运（形状+名次，带回不释放）

| 状态 | 动作 | 等待 | 成功 | 失败 |
|---|---|---|---|---|
| `D_PICK_ABOVE` | `Arm_MovePose(g_d_pick_above[rank])` | 动作完成 | `D_TRACK` | BUSY 重试 1 s；其他错误跳过 D |
| `D_TRACK` | `TrackStart(D, 50)` (0x42)；`Arm_NudgeTool(dx,dy)` 闭环 | 连续 3 样本 ≤5 mm | `TrackStop` (0x44) + `GetTcpPose` | 5 s/5X → 重粗调一次 → 仍失败跳过 D |
| `D_DESCEND` | `Arm_ApproachTool(+30 mm)` | 动作完成 | `D_GRASP` | 错误 → 回左展开、跳过 |
| `D_GRASP` | `Arm_Hold()`；等 600~800 ms；`ApproachTool(-30)` | 动作完成 | `D_LEFT_CARRY` | — |
| `D_LEFT_CARRY` | `Arm_MovePose(g_d_left_ready_carry)` 左侧展开，保持夹持 | 动作完成 | `D_FRONT_CARRY` | 错误 → 保持当前位姿出发 |
| `D_FRONT_CARRY` | `Arm_MovePose(g_d_front_carry)` 转回车体正面 | 动作完成 | `MS_RETURN` | 错误 → 保持当前位姿出发 |
| `MS_RETURN` | 选 `D_end` 启动，保持夹持 | `PATH_EV_DONE` | `MS_DONE` | 30 s → `MS_ERROR` |
| `MS_DONE` | **保持夹持，不 Release、不 Stow**；记录耗时 | 人工取走/上电复位 | — | — |

#### 2.5.1 执行顺序

**阶段 1：扫描与选目标（底盘运动中）**

1. C→D 表直行 335 → 右弧 90° → 直行 300，进入扫描段。
2. `PATH_EV_SCAN_BEGIN('D')` → `StartScan(D)` 0x48；**不发 0x40**；随即 `RequestOrder(D)` 0x49。
3. 扫描慢行 600 @100；视觉按**二维码第三位形状**筛目标，`0x4A` 返回名次 0/1/2。
4. `SCAN_READY` → 停 300 ms → 倒回 300 → `ARRIVE('D')`。
5. ARRIVE 当拍 `Notify('D')`；`rank = 0x4A`，无则 1。

**阶段 2：左侧展开 → 预存位 → 追踪抓取**

6. `Arm_MovePose(g_d_left_ready)`：左侧展开中转（空载）。
7. `Arm_MovePose(g_d_pick_above[rank])`：到对应形状目标的预存位。
8. `TrackStart(D, 50)` 0x42。
9. 每 100 ms：`GetTrack(D)`；`Arm_IsBusy()` 时跳过；否则 `Arm_NudgeTool(dx, dy, 2000)`。
10. 连续 3 个新鲜样本 `|dx|,|dy| ≤ 5 mm` → `TrackStop(D)` 0x44；`GetTcpPose`。
11. 5 s/5X → 回 `g_d_left_ready` → 重新 0x42 一次；仍失败跳过 D。
12. `Arm_ApproachTool(+30 mm)` → `Arm_Hold()` → 等 600~800 ms → `Arm_ApproachTool(-30 mm)`。

**阶段 3：持目标左展开 → 转正面 → 带回**

13. `Arm_MovePose(g_d_left_ready_carry)`：夹着目标回到左侧展开（持目标变体）。
14. `Arm_MovePose(g_d_front_carry)`：转回车体正面，保持夹持。
15. 确认 0x44 已发、无 pending 视觉事务；选 `D_end` 表启动。

**阶段 4：返回区**

16. 等 `PATH_EV_DONE`；30 s 超时 → `MS_ERROR`。
17. 到达返回区后**保持夹持，不 `Arm_Release()`、不 `Arm_Stow()`**，直接进入 `MS_DONE`。
18. 人工取走目标或上电复位；mission 不自动收臂。

#### 2.5.2 约束与数据缺口

- 目标：按二维码第三位形状筛；三个目标的预存位需实测。
- 中位线暂不发；底盘停车偏差直接转成目标预存位偏差。
- **D 只使用左侧展开，没有右侧展开**：空载用 `g_d_left_ready`，持目标用 `g_d_left_ready_carry`，随后转正面 `g_d_front_carry`；三套位姿需分别标定。
- z 先验高度同 B：目标高度相对机械臂底盘，工具 z 竖直向下；接近距离由 `ApproachTool` 处理。
- 追踪失败重粗调一次；仍失败跳过 D，空载回到正面/收起后前往返回区。
- **不释放**：到达返回区保持夹持；任务结束后机械臂不复位，除非人工介入。
- 返回区是否检查整车尺寸需确认；夹持目标可能使整车超出 250×250×300 mm。
- 数据缺口：0x4A 只有名次；0x42/0x43 只有平面 dx/dy；无 z、无夹取反馈；预存位与搬运位姿需实测。

### 2.6 A 点（2026-11-01 细化）

**A 扫码姿态（S20 释放扭矩实测 PWM）**

| 轴 | ID | PWM | 关节角 |
|---|---|---:|---:|
| J2 大臂 | 1 | **不动** | 保持当前 |
| J3 小臂 | 2 | **不动** | 保持当前 |
| J4 腕部 | 3 | **1559** | q4 = 0°（平伸） |

- `q3=164.3°` 需要 `ARM_Q3_MAX_DEG ≥ 166°`（已放宽）。
- J4 零位 1559 未重新标定，q4 换算仅作参考；若要精确复现，可直接按 PWM 下发。
- J1 在 A 点扫码时**右转 90°**；已实测 q1 正 = 左转，因此右转 90° 对应 **q1 = −90°**。

**执行时序**

1. 上电：陀螺零偏校准（静止 1 s + 3 s，OLED `GOK`）。
2. 张开夹爪（`ARM_GRIPPER_OPEN`，45° / 833 µs）。
3. A 扫码姿态：**只把腕部 J4 设为 1559（平伸），J2/J3 保持不动**，J1 保持 0。
4. 发车：表 0 起点 → A。
5. 到 A 停车（需在表 0 末尾新增 `PATH_TASK`，`point_id='A'`，`timeout_ms=15000`）。
6. J1 右转 90°（步进电机）。
7. 发送扫码指令 `0x10`，等待 `0x11` / `0x12`。
8. 扫码完成后按 §2.6.1 的重试/兜底规则处理。

**实现状态**：表 0 已加 `PATH_TASK('A')`；mission 已接入；J1 转向与 `0x10` 已串联；A 点只调腕部 J4=1559。

#### 2.6.1 原有规则

- 表 0 末尾新增 `PATH_TASK`，`point_id='A'`，`timeout_ms=15000`；路径进入该段时停车并发 `ARRIVE('A')`（上一段已完成制动，车已停稳）。
- **A 扫码姿态**：启动时张爪并只把腕部 J4 设为 1559（平伸），大臂 J2/小臂 J3 保持不动；与表 0 行驶并行。
- `g_arm_ready_pose` 数据待定，先以 `ARM_TEST_POSE` 占位；必须满足行驶安全与整车尺寸限制，并在实车标定后填入。
- 在线：`RequestScanCode()` 最多 **3 个事务**（每事务 1 s×3，事务间隔 500 ms）；`SCAN_OK` → `Notify('A')` → `MS_TO_B`。
- 失败计数：
  - 超时、`0x12(0x04 未检出)`、链路离线等待 1 s 超时，均计 1 次失败；
  - `0x12(0x01/0x02/0x03/0x06)` 视为不可恢复，**立即兜底**，不累计；
  - 连续 3 次失败 → **放弃所有任务**，置 `s_degraded=1`，`Notify('A')` → `MS_DEGRADED`。
- `PATH_TASK.timeout_ms=15000`：覆盖 3 个事务约 10 s + 5 s 余量；配置类错误会提前 `Notify`，不会等满。
- `MS_DEGRADED`：
  - 依次选 `A_B → B_C → C_D → D_end`；
  - 收到 `SCAN_READY` 忽略；收到 `ARRIVE` 当拍 `Notify`；
  - 不调用任何视觉命令、不请求中位线/顺序、不追踪、机械臂保持 Stow；
  - `D_end` 完成 → `MS_DONE`。
  - 扫描段仍按表慢行/停留/倒回，约多 5~8 s；若要求物理上不停车，需新增纯行驶降级路径表，不建议赛前临时改表。
- A 点对二维码的视野只靠表 0 停车位置保证；协议 0x10 没有偏移反馈。
- 二维码三位数由视觉端保存，MCU 只发 0x10、收 0x11。
- 未改表前的兜底：收到 `PATH_EV_DONE` 且无 ARRIVE 时，降级进入 `MS_A_TASK`；比赛构建必须用 `PATH_TASK`。

## 3. 事件与完成契约

- **唯一消费者**：`Mission_Update()`（运行在 MissionTask）独占 `Chassis_Path_TakeEvent()` 与 `Vision_Link_TakeEvent()`；`APP_STAGE == APP_STAGE_MISSION` 时 `app_debug` 不消费路径事件。
- **路径事件**：单槽改 4 深环形队列；`Start()` 不清空未消费事件。
- **视觉事件**：队列 8 → 16（`vision_link_config.h`），mission 每 5 ms drain；溢出记入 mission 日志。
- **优先级**：
  - P0 立即中止：`LINK_LOST`、关键事务 `TXN_ABORTED`、`PATH_EV_ERROR`、机械臂 COMM、总超时；
  - P1 当前状态结果：`SCAN_*`、`MIDLINE`、`ORDER`、`C_TARGET_DOWN`（需 zone/target/tick 匹配）；
  - P2 追踪异常：`TARGET_LOST`、`TRACK_LOST_5X`、`VISION_ERROR(5)`；
  - P3 信息：`LINK_RECOVERED`、`TX_ERROR`、其他错误码。
- **陈旧事件**：zone 不匹配、非当前状态、`C_TARGET_DOWN` 早于开激光时刻的事件一律丢弃并计数。
- **Notify 防护**：`Chassis_Path_NotifyTaskDone` 改为返回 int；只有 `Chassis_Path_IsWaitingTask()` 为真时调用；`s_notified_point` 锁存，同一点不重复通知。
- **完成事件锁存**：`Notify` 后路径会在同一拍发 `DONE`，此时 mission 可能尚未切到 `ZONE_PRESET`。事件泵必须把 `PATH_EV_DONE`/`PATH_EV_AUX_DONE` 写入 `s_done_latched`，各状态切换后先消费锁存，不能因为“当前状态不等待”而丢弃。
- **不要用 `Chassis_Path_IsRunning()` 判断整条路径结束**：它在 `PLAN_WAIT_TASK` 返回 0。

## 4. 需要新增的最小接口

### 路径层（经 `chassis.h` 暴露）

```c
/* 新增事件 */
PATH_EV_SCAN_BEGIN,   /* 扫描段开始，point_id='B'/'C'/'D'，用于发 0x28/0x38/0x48 */
PATH_EV_AUX_DONE,     /* 预设位/微转/伺服单步完成，与整表 DONE 区分 */

int Chassis_Path_StartPreset(uint8_t point_id, uint8_t rank);
int Chassis_Path_ServoStep(int16_t forward_mm, float turn_deg); /* C 底盘伺服单步 */
int Chassis_Path_NotifyTaskDone(uint8_t point_id);  /* 0=接受，-1=未等待/点号不符 */
```

- `StartPreset` 使用 `path_table.c` 新增的 B/C/D 各 3 张预设位表（距离占位，待实测）；距离为 0 时返回 1，mission 跳过。
- 中位线 2/3 不做底盘修正；`ScanSteer`/`NudgeSpin` 不再需要，不实现。
- `ServoStep` 用临时 `Path_Seg_t` 执行，完成发 `PATH_EV_AUX_DONE`；C 的转向与前后都由它承担。
- 所有新增接口在路径 RUNNING 或 WAIT_TASK 时按状态拒绝，避免与正常段冲突。

### 控制层（内部）

```c
void Chassis_Ctrl_AddHeadingOffset(float delta_deg);  /* clamp ±3° */
void Chassis_Ctrl_ClearHeadingOffset(void);
```

直行 PID 目标改为 `s_planned_heading + s_heading_offset`；扫描结束/Stop/BeginPath 清偏置。

### 视觉层

```c
uint8_t Vision_Link_CancelPending(void);  /* 取消扫码/中位线/顺序，不发帧 */
```

避免顺序事务 3 s pending 阻塞 TrackStart；同时修复审查中的 S1 缺陷（5X 清 active、0x12 关联事务、ACK/事件顺序）。

### 机械臂层

```c
Arm_Status_e Arm_GetTcpPose(Arm_Pose_t *out);
Arm_Status_e Arm_GetJoints(float q_deg[4]);
Arm_Status_e Arm_NudgeTool(int16_t dx_mm, int16_t dy_mm, uint16_t timeout_ms);
Arm_Status_e Arm_ApproachTool(int16_t dist_mm, uint16_t timeout_ms);
int          Arm_CheckPose(const Arm_Pose_t *pose);
uint8_t      Arm_LaserIsAvailable(void);
```

- `NudgeTool`：FK → 工具基向量 `dx·x + dy·y` → 新基座坐标 → IK → PTP；`|dx|,|dy| ≤ 80 mm`，死区不动作。
- `ApproachTool`：沿工具法向 n 平移。
- C 不使用 `NudgeTool`，只用 `GetTcpPose`/`LaserIsAvailable`。
- 修正 `arm_action.c` Aim 完成时自动开激光：改为 mission 在 `C_FIRE` 显式开，避免提前照射。

## 5. 文件与接线

新建：
- `include/app/mission.h` — Init/Update/Start/Abort/RequestAbort/Reset/IsActive/OwnsPath/GetView
- `include/app/mission_config.h` — 超时、容差、重试、位姿占位、`MISSION_AUTO_START_DELAY_MS`、`MISSION_DRY_RUN`
- `src/app/mission.c` — 顶层状态机、事件泵、A/返回、自动启动、故障
- `src/app/mission_zone.c` — B/C/D 共用扫描流程、区域状态、位姿表
- `include/arm/arm_cmd.h` — 机械臂命令类型、静态队列接口
- `src/arm/arm_cmd.c` — 8 深静态 `osMessageQueue`，`App_InitRtos` 创建

修改：
- `Core/Src/main.c` — 新增 MissionTask（1 KB、5 ms、Normal）；ArmTask 已有（1 KB、BelowNormal）
- `src/app/app_tasks.c` — 新增 `App_MissionTask()`：每 5 ms `Vision_Link_Update → Chassis_Path_Update → Mission_Update`；MainTask 按 `APP_STAGE` 裁剪职责
- `src/app/app_init.c` — `Mission_Init()`；`App_InitRtos()` 调 `Arm_Cmd_InitRtos()`
- `src/app/app_debug.c` — 阶段宏；MISSION 阶段不消费路径事件；500 ms 假完成用 `#if (APP_STAGE != APP_STAGE_MISSION)`；MISSION 页显示 mission 快照
- `src/app/app_tasks.h` — `App_MissionTask` 声明
- `src/chassis/planning/path.h/.c`、`path_table.c` — SCAN_BEGIN/AUX_DONE、预设位、ServoStep、A 点、fault/超时/seq
- `include/chassis/control/ctrl.h`、`src/chassis/control/ctrl.c` — 航向偏置、Submit 回滚、调试命令初始化
- `include/app/vision_link.h`、`src/app/vision_link.c` — `CancelPending` + S1 修复
- `include/arm/arm.h`、`src/arm/arm.c`、`src/arm/planning/arm_action.c` — 新接口、Aim 激光语义、`Arm_IsBusy/GetLastError` 改为临界区快照
- `include/arm/arm_config.h` — 新增 `ARM_TASK_PERIOD_MS`（5）与队列深度宏

不改：CubeMX 生成的外设初始化、`common/uart_rx`、协议文档。

### 5.1 任务堆与栈核算

- 7 个任务：512 B × 5（default/Main/Car/Enc/OLED）+ 1 KB × 2（Mission/Arm）= 4.5 KB 栈 + 约 0.7 KB TCB ≈ 5.2 KB，FreeRTOS 堆 8 KB，余量约 2.8 KB。
- MissionTask 1 KB 承载 path/vision/mission 调用链；ArmTask 1 KB 承载 `Arm_Motion_StartJoint` + 阻塞串口。
- MainTask 512 B 只扫键/调试/校准；MISSION 阶段不再进入机械臂调试分支。
- 验收时显示 `uxTaskGetStackHighWaterMark`，各任务剩余 ≥256 B。

## 6. 失败与降级

| 场景 | 重试 | 动作 | 继续 |
|---|---|---|---|
| A 扫码失败 | 连续 3 次（超时/0x04/离线各计 1 次）；配置类 0x01/02/03/06 立即兜底 | 放弃所有任务，进入 `MS_DEGRADED` 连续跑完五张表 | 是 |
| 中位线无回包（若启用） | 视觉层 500 ms 重发，mission 8 s | rank=1 | 是 |
| 顺序 3 s 兜底 | 0 | rank=1、flags=1 | 是 |
| 追踪 5X | 每次 1 次重粗调 | 停追踪 → 回观察位 → 重新中位线/追踪 | 仍失败跳过该区 |
| 机械臂 BUSY | 50 ms × 20 | 重试同动作 | 是 |
| 机械臂 TIMEOUT | 1 次 | Stop 后重试 | 仍失败跳过该区 |
| 机械臂 COMM/限位/不可达 | 0~1 次 | Stop/记录 | 跳过该区（J1 离线则中止） |
| 路径 fault/超时 | 0 | 停路径、停臂、关激光 | 中止 |
| 失联 | 0 | 停追踪、停臂、关激光、停路径 | 中止，人工复位 |
| 总耗时 270 s | 0 | D/返回阶段直接返回；其他阶段停车 | 中止/返回 |

## 7. 时间预算（占位距离，200 mm/s 巡航、100 mm/s 扫描）

| 阶段 | 标称 | 最坏 | 检查点 |
|---|---:|---:|---:|
| 起点→A | 14 s | 18 s | — |
| A 扫码 | 1.5 s | 8 s | A ≤40 s |
| A→B + B 任务 | 65 s | 82 s | B ≤120 s |
| B→C + C 任务 | 29 s | 37 s | C ≤170 s |
| C→D + D 任务 | 25 s | 36 s | D ≤235 s |
| D→返回 + 释放 | 11 s | 14 s | 返回 ≤280 s |
| 合计 | ≈146 s | ≈195 s | 上限 300 s |
| 降级跑图（A 后放弃任务） | ≈70 s | ≈90 s | 返回 ≤300 s |

路径距离若整体偏大，靠 A/B/C/D 检查点跳过保证返回。

## 8. 分阶段实施

**阶段 0：基础设施与任务拆分（不依赖实车）**
- 路径事件环、SCAN_BEGIN、AUX_DONE、预设位/ServoStep 接口、任务等待超时、fault 传播、seq 不重置、A 点 PATH_TASK；
- 新增 MissionTask（1 KB、5 ms、Normal）与 `App_MissionTask`；MainTask 按阶段裁剪职责；
- `arm_cmd` 静态队列、ArmTask 出队执行、`Arm_IsBusy/GetLastError` 快照化；
- 阶段宏 `APP_STAGE_CHASSIS/ARM/MISSION`；MISSION 自动启动与 6 中止；
- `APP_STAGE_ARM` 在命令队列完成前用 `#error` 阻止误编译。

验收：构建通过；A 点能停车发 ARRIVE；扫描段能发 SCAN_BEGIN；注入 fault 能中止；MISSION 上电自动进入 PREP；6 键能中止并停臂/停路径。

**阶段 1：mission 骨架 + A/B 最小闭环**
全部状态、事件泵、扫码、扫描/中位线/顺序、预设位、B 抓取/放置、失败降级、日志。验收：A→B 全流程跑通；事件无丢失；无死状态。

**阶段 2：C/D + 视觉/机械臂接口**
C 固定 Aim + 底盘伺服；D 追踪抓取 + 安全搬运；`Arm_NudgeTool/ApproachTool/GetTcpPose`；`Vision_Link_CancelPending`；Aim 激光语义。验收：C 伺服收敛 ±3 mm、激光时序正确；D 抓取方向正确。

**阶段 3：实车标定**
路径距离/扫描长度、B/C/D 预设位、tool_deg、中位线方向符号、C 伺服轴向与符号、追踪符号、夹爪 180/270、激光 GPIO。验收：连续 3 次全链 ≤240 s。

**阶段 4：异常与整链**
失联、5X、路径 fault、机械臂 COMM、超时；检查点跳过；5 分钟节拍。验收：注入故障仍能安全停车或返回。

## 9. 验证

- 构建无警告；7 个任务高水位各 ≥256 B；FreeRTOS 堆剩余 ≥2 KB。
- 全工程只有 MissionTask 调用 `Chassis_Path_TakeEvent`/`Vision_Link_TakeEvent`。
- ArmTask 是机械臂状态机唯一写者；mission/调试只通过 `arm_cmd` 入队。
- MISSION 阶段上电自动启动，6 键中止有效；复位需重新上电。
- Dry-run（可选 `MISSION_DRY_RUN=1`）：5 分钟内到达 DONE，无死状态。
- 串口助手注入视觉帧，验证 A 扫码三条路径、B/C/D 的 0x21/0x2A、追踪收敛、0x35。
- 实车：A→B→C→D→返回连续 3 次；失败注入后仍能降级完成。
- 尺寸与收拢姿态 ≤250×250×300 mm。

## 10. 待确认输入

1. **D 的终点**：已定抓取后回左侧展开 → 转正面 → 带回返回区，**保持夹持不释放**；返回区是否检查整车尺寸需确认。
2. **C 伺服轴映射**：已定**只前后、不转向**；`dy`→前后，`dx` 只记录。`K_move` 与符号需现场标定；若左右偏差大，再考虑启用转向。
3. **B 球与桶**：3 个颜色不同的球有顺序，桶只有 1 个；视觉端按二维码第一位颜色筛球并回名次。三个球的预存位、桶预存位、左/右展开位姿（空载/持球）需实测。若目标颜色球未检出，协议只允许回 1，MCU 无法区分“中位球”与“未检出”，需视觉端尽量用 `0x12(0x04)` 明确报错。
3b. **中位线**：B、C 点暂不发；D 是否启用待定。若不启用，底盘定位完全依赖路径表停车位置。
3c. **C 靶位**：按二维码第二位颜色筛靶，三个靶各一张预存位表；需实测。机械臂 Aim 固定一套位姿，激光出射点相对 TCP 的偏移需标定。
4. **A 二维码三位数**：已定由视觉端保存并用于 B 颜色/D 形状；MCU 只收 0x11。需视觉端保证 A 之后不丢失（重启/失联后能否恢复）。
4b. **A 点扫码失败**：已定连续 3 次失败（超时/未检出/离线）后放弃所有任务、连续跑完全图；配置类错误 0x01/0x02/0x03/0x06 立即兜底；A 点对准只靠表 0 停车位置。
5. **B/C/D 的 tool_deg 与 pick/place/aim/carry 位姿**：需主理人提供或实车示教。
5b. **A 点前展开位姿 `g_arm_ready_pose`**：数据待定；需确认行驶中展开是否超尺寸/碰撞，以及展开失败是否允许在 A 点重试。
5c. **B 点 z 先验高度**：已定 J1 旋转中心离地 **30.40 mm**（含底板偏移）；相对 J1 中心，球心 z=**4.60**、桶释放 z=**49.60**；D 人质中部 z=**104.60**。需确认夹取/放置时工具 z 竖直向下。
6. **启动/中止入口**：已定 MISSION 阶段上电自动启动；6=中止；复位需重新上电（或长按 6，待确认）。自动启动延时默认 1000 ms，需确认是否够人员离开。
7. **激光出射点与 TCP 的机械偏移**：默认并入 `g_c_aim` 标定。
8. **5 分钟计时起点**：默认从 `Mission_Start()` 开始。
9. **顺序名次 0/1/2 与现场摆放的对应**：需实测确认。
10. **C 目标倒下判定**：0x35 是否可靠；否则用固定 3 s。

---

## 11. 2026-11-01 规划变更：固定机械臂预存位 + 底盘移动补偿

### 11.1 新原则

1. **机械臂预存位姿固定**：每个任务点只用一套预存位，不随 rank 变化。
2. **位置差异由底盘前后移动实现**：rank 只决定底盘的倒退/前进距离。
3. **记录编码器实际移动距离**：每段底盘移动的距离都要记录，用于下一段起点补偿，防止误差累积。

### 11.2 新流程

#### B 点

1. 缓行扫描完成。
2. 按预存值选择**倒退距离**（由 rank 决定）。
3. 左展开机械臂。
4. 到机械臂**球预存位**（固定）。
5. 视觉伺服 → 夹取。
6. 夹取后回到左展开。
7. 按预存值前进到**排爆桶**（底盘移动）。
8. J1 **右转 180°**（已实测：q1 正 = 左转，右转 = q1 负）。
9. 机械臂**桶预存姿态**（固定）。
10. 视觉伺服 → 到位后放下。
11. J1 **回正**。
12. 收起机械臂。
13. 前进前往 C。

#### C 点

1. 扫描完成。
2. 按预存值**倒回目标距离**。
3. 展开机械臂。
4. 视觉伺服。
5. 期间记录行进距离（包括预存值距离）。
6. 完成任务 C 后，对**前往 D** 的距离进行补偿。

#### D 点

同 C：扫描 → 按预存值倒回 → 展开 → 视觉伺服 → 记录距离 → 对下一段补偿。

### 11.3 与原规划的差异

| 项 | 原规划（§2.3~2.5） | 新规划 |
|---|---|---|
| 机械臂预存位 | 按 rank 三套（球/靶/目标） | **一套固定位姿** |
| rank 的作用 | 选择机械臂预存位 | **选择底盘倒退距离** |
| B 桶位切换 | 右下展开中转 + 桶预存位 | **J1 右转 180° + 桶预存位** |
| 左右展开 | 空载/持球多套展开位姿 | 以**左展开**为主，桶位靠 J1 转向 |
| 底盘定位 | 固定路径表 / `StartPreset` 三张表 | **按预存值动态移动底盘** |
| 误差补偿 | 无 | **记录编码器距离，下一段起点补偿** |
| C 点 | 固定 Aim + 底盘前后伺服 | 展开机械臂 + 视觉伺服，记录距离补偿 |
| D 点 | 左侧展开 + 追踪 + 带回 | 同 C 的“按预存倒回 + 展开 + 伺服 + 补偿” |

### 11.4 需要的数据

1. B 点按 rank 的倒退距离（0/1/2）。
2. B 点到排爆桶的前进距离。
3. C/D 点按 rank 的倒回距离。
4. 机械臂固定预存位姿：球位、桶位、靶位、人质位。
5. 补偿规则：实际距离与预存值的差值，是加到下一段直行距离，还是修正路径起点。

### 11.5 需要的接口

- 按距离移动底盘（前进/倒退）并上报完成：类似 `Chassis_Path_ServoStep` 或 `StartPreset`。
- 记录本段编码器实际距离（`st.dist_done_mm` 已有）。
- 下一段距离补偿接口（路径表或控制层）。

### 11.6 B 点预存数据（2026-11-01）

- **扫描完回中的中点 = 预存中值（rank=1）**；基准倒回距离待实测。
- rank=0（近）：在基准倒回距离上**多后退 150 mm**。
- rank=2（远）：在基准倒回距离上**少后退 150 mm**。
- 排爆目标小球均在车体**左侧**（左展开取球）。
- 排爆桶与中值在**同一纵向位置的右侧**。
- 取球后前往桶的纵向距离换算（相对中值位置）：

| rank | 倒回偏差 | 到桶的前后移动 |
|---:|---:|---|
| 0（近） | 多后退 150 | **前进 150 mm** |
| 1（中） | 0 | 0（只需 J1 右转 180° 对准桶） |
| 2（远） | 少后退 150 | **后退 150 mm** |

- 桶在右侧，由 J1 右转 180° 对齐；底盘只负责纵向，不横向移动。

### 11.7 C/D 点预存数据（2026-11-01）

- 与 B 相同的 ±150 mm 规则：
  - 扫描回中值 = rank=1 基准；
  - rank=0（近）：多后退 **150 mm**；
  - rank=2（远）：少后退 **150 mm**。
- **C 靶、D 人质均在车体左侧**（与 B 的目标球同侧）；B 的排爆桶在右侧。
- 因此 B 需要取球后 J1 右转 180° 对桶；C/D 全程使用左侧，不需要转向对侧。
- C 完成后按记录的编码器实际距离对前往 D 的路径补偿；D 同理对返回段补偿。

### 11.8 B/C/D 通用动作顺序（2026-11-05 更新）

1. 直接行驶到任务区中间停车（原扫描段的中点），不再慢行扫描。
2. **先展开机械臂**（左展开），再向视觉请求目标名次。
3. 收到名次后按 rank 用 `ServoStep` 前后移动（rank0 −150、rank1 0、rank2 +150）。
4. 进入任务机械臂预存态 → 视觉伺服 → 任务动作。

> 旧实现为“慢行 700 + 自动倒回 350 + 再退 150”三段动作，2026-11-05 已取消；路径表末段改为停车 `PATH_TASK`。

### 11.9 展开位（2026-11-01）

- **左展开位**：B/C/D 三个任务**共用同一套**机械臂位姿（J2/J3/J4 三个 PWM 相同），只需标定一次。
- **右展开位**：三个舵机数值与左展开**完全相同**，仅 **J1 步进方向相反**（转向另一侧）。
  - 即右展开不需要另一套舵机标定，靠 J1 转向实现镜像。
- B 点取球后到桶位的动作：左展开 → 按预存前进到桶 → J1 右转 180°（即右展开）→ 桶预存姿态。

### 11.10 演示模式与正式模式（2026-11-01）

- **演示模式**：无视觉上位机，按预存位姿和固定时序完成动作。
- **正式模式**：含视觉上位机，按协议做扫描/伺服/追踪。

演示模式机械臂位姿（PWM，ID1/J2、ID2/J3、ID3/J4，已存入 `arm_config.h`）：

| 名称 | J2 | J3 | J4 |
|---|---:|---:|---:|
| 通用左展开 | 1960 | 1502 | 868 |
| B 机械臂预存 | 1595 | 1504 | 1055 |
| B 演示机械臂（取球） | 1443 | 1642 | 1214 |
| B 演示放置桶 | 1756 | 1677 | 994 |
| B 放置桶机械臂预存（正式） | 1959 | 1642 | 1004 |
| C 预存（演示通用） | 1427 | 1680 | 1864 |
| D 机械臂预存 | 1842 | 1642 | 1175 |
| D 演示机械臂 | 1619 | 1573 | 1153 |

### 11.11 演示模式规则（2026-11-01）

1. **串行执行**：一个动作完成后再做下一个。
2. **底盘只以中值为目标**（rank=1），不做 ±150 调整。
3. **展开**：先机械臂到通用展开位，再步进转向对应方向。
4. **收起**：先步进回正，再机械臂收起。
5. **每个任务完成后回到对应方向的展开态**：B 回右展开，C/D 回左展开。
6. **B/D 使用夹爪，C 不使用夹爪**。
7. **D 回收起时夹爪保持夹取**（不释放）。
8. **激光（C）开 2 秒**后关闭。
9. **夹爪动作与第一个机械臂动作同步，第一个动作是夹爪收/放**。

### 11.12 演示模式完整流程

**夹爪状态区分**：张开 `OPEN`（45°/833 µs）、**夹取 `GRASP`（105°/1278 µs）**、闭合 `CLOSE`（135°/1500 µs）。演示模式取目标用 **GRASP**，不是 CLOSE。

**时序常量**：机械臂每个动作 **3 s**；机械臂与步进之间间隔 **2 s**；激光 **2 s**；夹爪动作与第一个机械臂动作**并行**。

**B 点**

1. 到点（中值，rank=1，不前调 ±150）。
2. **夹爪张开**，同时机械臂去通用左展开(1960/1569/868)，3 s。
3. 等 2 s。
4. 步进保持左侧（q1=0）。
5. 机械臂到 B 演示取球(1443/1737/1214)，3 s。
6. 夹爪 **夹取**（GRASP 105°），等 0.8 s。
7. 机械臂回通用左展开，3 s。
8. 底盘移动到桶的纵向位置（中值前后差 0）。
9. 步进右转 180°（q1 −180），等 2 s。
10. 机械臂到 B 演示放置桶(1756/1778/994)，3 s。
11. 夹爪张开（OPEN 45°），等 0.8 s。
12. 机械臂回通用展开（仍为右），3 s。
13. 等 2 s。
14. 步进回正（q1 0），等 2 s。
15. 机械臂收起（Stow），3 s。
16. 前往 C。

**C 点**

1. 到点（中值）。
2. 机械臂到通用左展开，3 s。
3. 等 2 s。
4. 步进保持左侧。
5. 机械臂到 C 预存(1427/1680/1864)，3 s。
6. 激光开 **2 s** → 关。
7. 机械臂回通用左展开，3 s。
8. 等 2 s。
9. 步进回正（q1 0），等 2 s。
10. 机械臂收起，3 s。
11. 前往 D。

**D 点**

1. 到点（中值）。
2. **夹爪张开**，同时机械臂去通用左展开，3 s。
3. 等 2 s。
4. 步进保持左侧。
5. 机械臂到 D 演示(1619/1654/1153)，3 s。
6. 夹爪 **夹取**（GRASP 105°），等 0.8 s。
7. 机械臂回通用左展开，3 s。
8. 等 2 s。
9. 步进回正（q1 0），等 2 s。
10. 机械臂收起，3 s，**夹爪保持夹取，不释放**。
11. 返回。

### 11.13 正式模式数据决策（2026-11-01）

- **C 伺服 `dy→前后` 的 K 先用 1.0 占位**，符号待实车标定（`forward = K × dy`）。
- **C 使用预存位，不引入 `tool_deg`**。
- **B/D 的 `tool_deg` 先用 −45°（斜向下 45°）占位**，正式标定前不改。
- **路径距离沿用 `path_table.c` 当前值**，不再单独标定。
- **B/D 追踪 `dx/dy` 符号按协议 §2 定义**（工具系 x 左、y 上、z 前，右手系），无需实车试符号；只需实现 `R_tool→base` 变换。
- **J4 零位已确认可用**，不再列为待办。

---

## 12. 正式模式完整流程与兜底（2026-11-01）

### 12.1 全局约定

- 视觉在线判定：500 ms 内收到任意合法帧；失联先停追踪、关激光并等待恢复；**超过 5 s 未恢复 → J1 回正 → 收起机械臂 → 降级跑完全图**。
- 路径事件：`SCAN_BEGIN`、`SCAN_READY`、`ARRIVE`、`AUX_DONE`、`DONE`、`ERROR`。
- 视觉事件：`SCAN_OK/ERROR/TIMEOUT`、`MIDLINE`、`ORDER`、`TRACK`、`TARGET_LOST`、`TRACK_LOST_5X`、`LINK_LOST/RECOVERED`。
- 机械臂命令：经 `arm_cmd` 队列，ArmTask 单写；完成/错误轮询快照。
- 高度：**由 MCU 按预存位与场地高度数据决定**（球心 4.60、桶 49.60、人质中心 104.60 mm 等，见 `arm_config.h` §10），视觉只提供平面 `dx/dy`。
- 中位线：**已启用**，B/C/D 各请求两次（展开后首次对准、按 rank 移动后复核），命令 `0x20/0x30/0x40`。
  - 判定值：`0` 未检出 / `1` 已进入范围 / `2` 偏左 / `3` 偏右。
  - **方向约定（已确认）**：`2` 偏左 → 底盘**后退** 30 mm；`3` 偏右 → 底盘**前进** 30 mm。
  - **每 500 ms 发一帧请求**，底盘按最近判定以 **50 mm/s 连续低速前后移动**，直到收到 `1`。
  - **5 s 总超时**或**连续 3 个窗口无响应**（约 1.5 s）即原地停车，继续任务。
  - **位移记录**：路径层 `AUX_DONE` 带回实际位移并记入 `s_align_mm`，仅作记录；**rank ±150 与到桶换算不变**，中位线只负责微调。
- 顺序兜底：3 s 无回包 → rank = 1。
- 末端位姿上报 `0x14`：**已删除**，MCU 不再向视觉发送 TCP 位姿。
- 追踪参数：50 Hz，死区 2 mm，限幅 ±80 mm；连续 3 个新鲜样本 ≤5 mm 视为收敛。
- 夹爪：无回读，动作后固定等待 600~800 ms。
- 总耗时检查点：A ≤40 s、B ≤120 s、C ≤170 s、D ≤235 s、返回 ≤280 s。

### 12.2 A 点

**正常流程**

1. 上电陀螺校准 → 张爪 45° + 腕部 J4=1559（并行）→ 发车表 0。
2. 表 0 末尾 `PATH_TASK('A')` 停车 → `ARRIVE('A')`。
3. **不动 J1**，只保持腕部姿态；到点后先停稳 **500 ms**，再发 `Vision_Link_RequestScanCode()`（0x10），3 s 计时从发送时刻开始。
4. 收到 `0x11` → **启动闭爪 + 收起（不等待）** → `Notify('A')` → 立即发车表 1；收臂与表 1 的 CALIB 并行。
   扫码等待期间机械臂保持扫码姿态，不提前回收。
5. 进入 B 点前可保持 A 扫码姿态，或按 B 流程先展开。

**失败与兜底**

| 情况 | 处理 |
|---|---|
| 0x10 发出后 3 s 无响应 | 判扫码失败，**启动回收** → `Notify('A')` → **交给演示流程**（`MS_DEMO`，直接选表 1 继续跑 B/C/D，回收与行驶并行），**不重试** |
| 0x12（任意错误码） | 与超时同样处理，直接交给演示流程 |
| 路径 fault / 超时 | 停路径、停臂，进入 ERROR |

> 扫码 `0x10` **只发一帧**（`VISION_LINK_SCAN_MAX_TX=1`），发出后 3 s 无响应判失败（`VISION_LINK_SCAN_RESPONSE_TIMEOUT_MS=3000`）；mission 侧 `MISSION_SCAN_CODE_TIMEOUT_MS=3000` 作同刻兜底。
> 进入演示后由 `App_Debug_StartDemoFromA()` 接管：`state 3` J1 回正 → `state 4` 闭爪+收起 → `state 5` 表 1 → B/C/D 演示动作。MISSION 阶段按 `7` 可从起点直接模拟同样的演示流程。

**降级（MS_DEGRADED）**

> A 点扫码失败已改为交给演示流程，不再进入本状态。本状态仅保留给**视觉失联超过 5 s** 的场景。

- 依次跑 A_B → B_C → C_D → D_end，不做任何任务动作；
- 机械臂保持收起；
- 扫描段仍按表行驶，收到 ARRIVE 当拍 Notify；
- 只保证 5 分钟内回到返回区。

### 12.3 B 点

**正常流程**

1. 表 1：CALIB（停车重标）→ **后退 300 → 左弧 90° → 直行 130 → 右弧 90° → 直行 340@150（过减速带）→ 左弧 90° → 直行 280 → 右弧 90° → 直行 350 → 右弧 90° → 直行 20 → 直行 350 到任务区中间停车**。距离为当前实车调整值。
2. 到点 `ARRIVE('B')` → 当拍 `Notify('B')`，路径空闲。
3. 机械臂：通用左展开（`MISSION_POSE_COMMON_OPEN`）→ J1 左转 90°。
4. 展开到位后 `StartScan(B)` 0x28。
5. **首次中位线对准**：每 500 ms 请求 0x20 → 0x21；`2` 后退、`3` 前进，50 mm/s 连续低速移动，`1` 即停；5 s 或连续 3 次无响应即原地停车。
6. `RequestOrder(B)` 0x29，等 `0x2A`（3.5 s 兜底 rank=1）。
7. 按 rank 用 `ServoStep` 前后移动底盘：**rank0 −150、rank1 0、rank2 +150**（停车点在中间，rank0 已被驶过需后退）。
8. **再次中位线对准**（0x20），复核底盘与目标的对准。
9. 机械臂到 B 预存位 → `TrackStart(B_OBJECT, 50)` 0x22。
8. 每 100 ms：`GetTrack(B)`；`Arm_IsBusy()` 时跳过；否则 `Arm_NudgeTool(dx, dy, 2000)`（按协议 §2 工具系）。
9. 连续 3 个新鲜样本 ≤5 mm → `TrackStop` 0x24。
10. `ApproachTool(+30)` → `Hold()` → 等 600~800 ms → `ApproachTool(-30)`。
11. 回左展开 → 底盘按 rank 前后换算到桶（rank0 前进 150、rank1 0、rank2 后退 150）→ J1 右转 180°。
12. 机械臂到 B 放置桶预存 → `TrackStart(B_BUCKET, 50)` 0x25 → 伺服 → `TrackStop` 0x27。
13. `ApproachTool(+20)` → `Release()` → 等 600~800 ms → `ApproachTool(-20)`。
14. 回右展开 → J1 回正 → 收起 → 表 2。

**失败与兜底**

| 情况 | 处理 |
|---|---|
| 0x28/0x29 发送失败 | 100 ms 重试，最多 10 次；仍失败跳过 B 视觉，直接执行底盘动作 |
| 0x2A 3 s 无回包 | rank=1，继续 |
| 0x2A 未检出（协议回 1） | 按 rank=1 处理，可能抓错球 |
| 物体追踪 5 s 未收敛或 5X | 停追踪 → 回展开/观察位 → 重试一次 → 仍失败跳过 B |
| 桶追踪失败 | 用名义桶位直接放置；仍失败原地释放 |
| 机械臂 BUSY | 50 ms × 20；TIMEOUT 停后重试 1 次；COMM：J1 离线中止，否则跳过 B |
| 夹爪无反馈 | 固定等待 600~800 ms |
| B 总超时（ARRIVE 起 30 s） | 停追踪、回展开、跳过 B |
| 离开 B 前 | 必须确认 0x24/0x27 已发、无 pending 视觉事务、夹爪安全、机械臂收起 |

### 12.4 C 点

**正常流程**

1. 表 2：直行 1400 → 直行 350 到任务区中间停车。
2. 到点 `ARRIVE('C')` → 当拍 `Notify('C')`。
3. 机械臂通用左展开 → J1 左转 90°。
4. 展开到位后 `StartScan(C)` 0x38。
5. **首次中位线对准**：0x30 → 0x31，每 500 ms 一帧，50 mm/s 连续低速移动，`1` 即停；5 s 或连续 3 次无响应即停车。
6. `RequestOrder(C)` 0x39，等 `0x3A`（3.5 s 兜底 rank=1）。
7. 按 rank `ServoStep` 前后移动（rank0 −150、rank2 +150）。
8. **再次中位线对准**（0x30）复核。
9. 机械臂到 C 预存（固定，不随 rank 变）→ `TrackStart(C, 50)` 0x32。
7. 每 200 ms：`GetTrack(C)`，只取 `dy`；`forward = K × dy`，K=1.0 占位，限幅 ±20 mm/步 → `Chassis_Path_ServoStep(forward, 0)`；等 `AUX_DONE`。
8. 连续 3 样本 `|dy| ≤ 3 mm` 或 4 s → `TrackStop` 0x34。
9. `Arm_Laser(1)` → 等 `0x35` 或 3 s → `Arm_Laser(0)`。
10. 回展开 → J1 回正 → 收起 → 表 3。

**失败与兜底**

| 情况 | 处理 |
|---|---|
| 0x3A 无回包 | rank=1 |
| 追踪 4 s 未收敛 | 用最后位置停下，仍执行激光 |
| 底盘伺服单步超时（2 s） | 跳过本步，不再修正 |
| 0x35 不可靠 | 固定 3 s 超时关激光 |
| 机械臂 Aim/预存失败 | 记录降级，跳过 C |
| 激光不可用（`Arm_LaserIsAvailable()==0`） | 记录降级，跳过开激光 |
| 路径 fault/伺服超时 | 关激光、跳过 C |

### 12.5 D 点

**正常流程**

1. 表 3：直行 305 → 右弧 90° → 直行 300 → 直行 300 到任务区中间停车。
2. 到点 `ARRIVE('D')` → 当拍 `Notify('D')`。
3. 机械臂通用左展开 → J1 左转 90°。
4. 展开到位后 `StartScan(D)` 0x48。
5. **首次中位线对准**：0x40 → 0x41，每 500 ms 一帧，50 mm/s 连续低速移动，`1` 即停；5 s 或连续 3 次无响应即停车。
6. `RequestOrder(D)` 0x49，等 `0x4A`（3.5 s 兜底 rank=1）。
7. 按 rank `ServoStep` 前后移动（rank0 −150、rank2 +150）。
8. **再次中位线对准**（0x40）复核。
9. 机械臂到 D 机械臂预存 → `TrackStart(D, 50)` 0x42 → `Arm_NudgeTool(dx,dy)` 闭环。
7. 连续 3 样本 ≤5 mm → `TrackStop` 0x44。
8. `ApproachTool(+30)` → `Hold()` → 等 600~800 ms → `ApproachTool(-30)`。
9. 回左展开 → J1 回正 → 收起（**保持夹取，不释放**）→ 表 4 → 终点。

**失败与兜底**

| 情况 | 处理 |
|---|---|
| 0x4A 无回包 | rank=1 |
| 追踪 5 s/5X | 重粗调一次 → 仍失败跳过 D |
| 机械臂动作失败 | 回展开、跳过 D |
| 夹取失败 | 跳过 D，空载带回 |
| 返回超时（30 s） | ERROR，人工处理 |
| 到达终点 | 保持夹持，不 Release、不 Stow，等待人工取走 |

### 12.6 全局异常

| 异常 | 动作 |
|---|---|
| 视觉失联（500 ms） | 停追踪、关激光；等待恢复；**超过 5 s 未恢复 → J1 回正 → 收起机械臂 → 降级跑完全图（不做任务）** |
| 路径 fault | 停臂、停路径、关激光 → ERROR |
| 机械臂 COMM（J1 离线） | 中止整链 |
| 机械臂其他错误 | 记录、跳过当前任务 |
| 总耗时检查点超限 | 跳过后续任务，直接返回 |
| 按键 6 急停 | 停路径、停臂、关激光、停追踪 → ABORT |
| 复位 | 重新上电；不自动重启 |

### 12.7 实现顺序

```
1. 路径层：PATH_EV_SCAN_BEGIN、PATH_TASK('A')、事件环、fault 传播、任务等待超时
2. 路径层：按 rank 一次性倒回（动态距离）
3. 机械臂：Arm_NudgeTool / Arm_ApproachTool / GetTcpPose
4. 路径层：Chassis_Path_ServoStep + AUX_DONE
5. 任务层：mission 状态机（或扩展调试状态机）消费视觉/路径事件
6. 视觉层：S1 六项修复、CancelPending、事件队列 16
7. 联调 A → B → C → D
```

---

---

