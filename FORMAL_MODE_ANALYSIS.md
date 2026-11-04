# 正式模式实现与静态验证报告

> 日期：2026-11-01
> 范围：正式模式（含视觉上位机）全部代码
> 状态：**代码完成、构建通过、未做实车验证**

---

## 1. 实现清单

### 1.1 路径层（`path.c` / `path.h` / `path_table.c`）

| 项 | 内容 |
|---|---|
| 事件类型 | 新增 `PATH_EV_SCAN_BEGIN`、`PATH_EV_AUX_DONE` |
| 事件队列 | 单槽 → 4 深环形队列（满时丢最旧） |
| 扫描开始 | `PATH_SCAN` 段启动时发 `SCAN_BEGIN(point_id)` |
| A 点 | 表 0 末尾新增 `PATH_TASK('A')`，`timeout=15 s` |
| 按 rank 倒回 | `Chassis_Path_SetRank()`；rank0 +150、rank2 −150、rank1 中值 |
| 底盘伺服 | `Chassis_Path_ServoStep(forward, turn)`，完成发 `AUX_DONE` |
| fault 传播 | 段执行中 `st.fault` → `PATH_EV_ERROR` |
| 任务超时 | `SEG_TASK_WAIT` 按段 `timeout_ms` 超时 |
| Notify | `Chassis_Path_NotifyTaskDone()` 返回 int（0 接受 / -1 不符） |

### 1.2 机械臂层（`arm.c` / `arm.h`）

| 接口 | 说明 |
|---|---|
| `Arm_GetJoints` / `Arm_GetTcpPose` | 关节/FK 查询 |
| `Arm_NudgeTool(dx,dy)` | 协议 §2 工具系（x 左、y 上、z 前）→ 基座变换 → IK → PTP |
| `Arm_ApproachTool(dist)` | 沿工具法向平移 |
| `Arm_MovePwm(pwm,time)` | 三舵机 PWM 同步帧 |
| `Arm_MoveBaseRelative(deg)` | J1 相对旋转，正=左转 |
| `Arm_Grasp` / `Arm_Close` | 夹取 105° / 闭合 135° |

工具系变换：

```
y_tool = (-sinφ·cosq1, -sinφ·sinq1, cosφ)
x_tool = (-sinq1, cosq1, 0)
z_tool = ( cosφ·cosq1,  cosφ·sinq1, sinφ)
φ = q2 − q3 + q4
```

### 1.3 视觉层（`vision_link.c` / `vision_link_config.h`）

- 事件队列 8 → **16**；
- 事件满时丢弃最旧、保证新事件入队；
- `0x12` 按 pending 类型处理：扫码 / 中位线 / 顺序（顺序按 rank=1 兜底）；
- `TRACK_LOST_5X` 后清 `s_track_active`，允许同目标重新 TrackStart；
- 新增 `Vision_Link_CancelPending()`。

### 1.4 任务层（新增 `mission.c` / `mission.h` / `mission_config.h`）

- `APP_STAGE_MISSION` 时由 MainTask 调用 `Mission_Update()`；
- mission 独占路径与视觉事件；
- 状态：`PREP → A_TRAVEL → A_TASK → TO_B → B_TASK → TO_C → C_TASK → TO_D → D_TASK → RETURN → DONE`，另有 `DEGRADED / ERROR / ABORT`；
- A：张爪+摆臂+发车 → ARRIVE → J1 右转 90° → 0x10 → 成功/重试/降级 → J1 回正 → 闭爪+收起 → 表 1；
- B：展开 → J1 左转 90° → 预存 → 0x22 追踪 + NudgeTool → 夹取 → 回展开 → 按 rank 到桶 → J1 180° → 桶预存 → 0x25 → 放下 → 回正 → 收起 → 表 2；
- C：展开 → J1 左转 90° → C 预存 → 0x32 + ServoStep（K=1.0）→ 0x34 → 激光 3 s/0x35 → 收起 → 表 3；
- D：展开 → J1 左转 90° → D 预存 → 0x42 + NudgeTool → 夹取 → 回展开 → 回正 → 收起（保持夹取）→ 表 4 → 终点；
- 失联 500 ms 停追踪/关激光；超 5 s → J1 回正 → 收起 → 降级跑图；
- 按键 6 → `Mission_RequestAbort()`，其余调试键在 MISSION 阶段禁用。

---

## 2. 构建验证

| 阶段 | 结果 | RAM | Flash |
|---|---|---:|---:|
| `APP_STAGE_CHASSIS` + S14 | 通过 | 15580 B (23.8%) | 63212 B (12.1%) |
| `APP_STAGE_MISSION` | 通过 | 15604 B (23.8%) | 67108 B (12.8%) |

无编译警告。

---

## 3. 静态审查发现

### 3.1 已覆盖

- 事件生产者/消费者唯一性：MISSION 阶段只有 mission 消费路径/视觉事件；
- A 点 PATH_TASK 与 Notify 契约；
- 按 rank 一次性倒回；
- fault 传播、任务等待超时；
- 追踪 5X 后可重发；
- 0x12 关联事务；
- 事件满不丢最新。

### 3.2 遗留风险（需实物或后续处理）

| # | 风险 | 说明 |
|---|---|---|
| 1 | 视觉 stop/start 无送达确认 | 0x24/0x27/0x34 丢失时视觉会继续推流，需协议层重发 |
| 2 | ACK 先清位后发送 | 发送失败时 ACK 丢失，未改 |
| 3 | 响应/超时同拍竞态 | pending 超时检查前未再 drain 一次 |
| 4 | 在线判定过宽 | 任意合法帧刷新在线，未限定命令/心跳 |
| 5 | 0x2A 未检出=1 不可区分 | 视觉端需尽量发 0x12(0x04) |
| 6 | B 追踪重试不回展开 | 直接回预存位重试，非文档的“回观察位” |
| 7 | DEGRADED 从表 1 重新开始 | 若在 B/C/D 失联，可能重复已跑表 |
| 8 | 机械臂 PWM 动作无完成反馈 | 全部用固定延时，实车需复核时间 |
| 9 | J1 无真实回读 | 转向后软件累计，丢步不可检测 |
| 10 | C 伺服 K=1.0 占位 | 方向/比例待实测 |
| 11 | B/D tool_deg=−45° 占位 | 位姿精度依赖预存 PWM |

### 3.3 未改动项

- `App_Debug_HandlePathEvents` 的 500 ms 假完成仍存在于 CHASSIS 阶段；
- vision_link 的 S1 中第 1/2/3 项（ACK 时序、同拍竞态、在线判定）未改；
- 中位线（0x20/0x30/0x40）未启用。

---

## 4. 无实物条件下的验证方案

1. **串口注入帧**：用串口助手按协议向 USART3 注入 `0x11`、`0x12`、`0x2A`、`0x23`、`0x35`、`0xFE`，观察 `Vision_Link_TakeEvent` 与 mission 状态；
2. **路径事件模拟**：临时在调试页注入 `PATH_EV_ARRIVE_POINT` / `SCAN_BEGIN` 验证状态机分支；
3. **静态检查**：构建、`arm-none-eabi-size`、grep 未使用符号；
4. **桌面单元测试**：为 XOR、帧解析、事件环写 host 测试（可选）。

---

## 5. 切换方式

```c
/* include/app/app_debug.h */
#define APP_STAGE  APP_STAGE_MISSION   /* 正式模式 */
#define APP_STAGE  APP_STAGE_CHASSIS   /* 演示/底盘调试 */
```

- 正式模式：上电校准后 1 s 自动 `Mission_Start()`；按键 6 中止；
- 演示模式：S14 键 9 走演示状态机。

---

## 6. 结论

正式模式代码链路已完整：路径事件、视觉事务、机械臂伺服、任务编排与降级均已落地，两个阶段构建通过。剩余工作集中在视觉端联调、stop 送达确认和若干占位标定值，均需要在实物或视觉端参与下完成。
