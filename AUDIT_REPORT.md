# 26GX_New 全工程落地审查报告

> 审查日期：2026-10-27
> 范围：`src/`、`include/`、`Core/Src/`、`platformio.ini`、全部文档与协议
> 状态：**进行中**。任务链/逻辑链接已完成；底盘/机械臂、协议/工程两份审查进行中。
> 说明：本报告只记录静态审查证据与判断，不替代实车验证；所有结论均给出文件与行号。

---

# 一、任务链、逻辑链接与并发（已完成）

## 1. 完整任务流程调用链

`App_MainTask` 是唯一业务编排任务，5 ms 节拍，优先级 Normal（[app_tasks.c:59-98](src/app/app_tasks.c#L59-L98)）：

```
Keypad_Scan
  → App_Debug_HandleKey
  → App_Debug_Update
  → Vision_Link_Update
  → Chassis_Path_Update
  → App_Debug_HandlePathEvents
  → Arm_Update
```

- **直接调用**：MainTask → Keypad / App_Debug / Vision_Link / Chassis_Path / Arm_Update；Path → Control（状态读取 + 命令队列）；Arm_Update → 步进/舵机/反馈/动作层。
- **事件/队列**：Path → app 为**单槽事件**；Vision → app 为 **8 深事件环但无人消费**；MainTask → CarTask 为 `osMessageQueue`；Control → 完成队列**无人消费**。
- **完全没有连接**：Vision ↔ Path/Arm/Mission；Path 事件 ↔ Mission；Arm 完成 ↔ 上层；Chassis fault ↔ Path/Mission；Mission 本身不存在。

关键证据：
- MainTask 调用顺序 `app_tasks.c:84-96`，周期 `KEYPAD_SCAN_PERIOD_MS` 于 `:81`。
- MainTask 512 B 栈、优先级 `Core/Src/main.c:64-68`，运行时调整 `:197-200`。
- Path 读控制状态 `path.c:454-468`；下发命令 `path.c:212`；控制队列接收 `ctrl.c:151`、`:564-566`。
- Path 单槽事件 `path.c:48`、`:106-111`、`:512-522`。
- 唯一消费者 `app_debug.c:161-197`，由 `app_tasks.c:93` 无条件调用。
- Vision 注册 `app_init.c:14-22`，更新 `app_tasks.c:89`，ISR 注册 `vision_link.c:1088`。
- Arm 更新 `arm.c:66-74`；动作层 `arm_action.c:379-420`。
- 无 mission 文件：`src/app/` 只有 `app_debug.c`、`app_init.c`、`app_tasks.c`、`vision_link.c`。

## 2. 路径事件链路

- **生产者**：`path_emit()` 直接覆盖写 `s_event`。
- **消费者**：只有 `App_Debug_HandlePathEvents()`；`Chassis_Path_TakeEvent()` 单槽取后清零。
- **当前消费逻辑**（`app_debug.c:161-197`）：
  - `ARRIVE_POINT` → 记录 `point_id`，500 ms 后 `NotifyTaskDone`；
  - `DONE` → 调试链式标志有效时 2 s 后切下一张表；
  - `SCAN_READY` / `ERROR` → 只写 `s_last_path_ev`，**无任何处理**，且该变量从未被读取。

### 与未来 mission 的冲突

1. **事件被 debug 抢走**：debug 每拍无条件清空单槽，mission 若在其后永远看不到事件。
2. **假完成与真实任务竞争**：debug 会在 mission 完成前 `NotifyTaskDone`；mission 再通知会留下陈旧 `s_task_done_id`，下一次同名任务点被瞬间放行。
3. **双链式启动竞争**：debug 的 `s_chain_active` 会在 DONE 后自动切表；mission 若也切表会重复 `Select/Start`，且 `Start` 清空事件与完成标志。
4. **单槽覆盖**：`SCAN_READY` 未消费就被后续 `ERROR` 覆盖；陈旧通知导致 ARRIVE 被 DONE 覆盖。
5. **NotifyTaskDone 契约过弱**：单字节、无队列、无来源/表号/运行号校验、无返回值；`point_id == 0` 永远无法放行；`SEG_TASK_WAIT` **无超时**。
6. **启动清空**：`Chassis_Path_Start()` 清事件与完成标志，Start 前的通知丢失。

证据：`path.c:48`、`:106-111`、`:121-134`、`:369-387`、`:496-504`、`:512-527`。

### A 点没有事件

- 表 0 `to_A` 的 7 段全部 `point_id = 0`，无 `PATH_TASK`、无 `PATH_SCAN`（`path_table.c:17-26`）。
- 路径结束只发 `PATH_EV_DONE`，`point_id` 固定 0（`path.c:121-134`）。
- B/C/D 的到达事件来自"扫描段走完 → 倒回中心 → ARRIVE"（`path.c:285-307`、`:457-462`）。
- 若给 A 加 `PATH_TASK`，必须用非零 `point_id`（如 `'A'`），否则 `path.c:498` 的 `s_task_done_id != 0U` 永不成立。

### 五张表的 point_id 分布

| 表 | 段与 point_id | 事件 |
|---|---|---|
| 0 `to_A` | 7 段全部 0 | 只有 DONE |
| 1 `A_B` | CALIB 0、直行 0、弧线 0、直行 0、弧线 0、SCAN `'B'` | SCAN_READY('B')、ARRIVE('B')、DONE |
| 2 `B_C` | 直行 0、SCAN `'C'` | SCAN_READY('C')、ARRIVE('C')、DONE |
| 3 `C_D` | 直行 0、弧线 0、直行 0、SCAN `'D'` | SCAN_READY('D')、ARRIVE('D')、DONE |
| 4 `D_end` | 直行 0 | 只有 DONE |

所有表都没有 `PATH_TASK`；`PATH_STOP` 从未使用（`path_table.c:17-65`）。

### debug 的 500 ms / 2000 ms 逻辑

- `APP_DEBUG_TASK_MS = 500U`（`app_debug.h:50-51`），到期通知（`app_debug.c:191-196`）。
- `APP_DEBUG_CHAIN_DWELL_MS = 2000U`（`app_debug.h:53-54`），DONE 后切表（`app_debug.c:175-188`），定时启动（`:303-310`）。
- 链式入口按键 9（`:526-533`）。

## 3. 视觉链路接入后的缺口

`Vision_Link_Init()` 已注册，`Vision_Link_Update()` 已每 5 ms 运行；但所有命令/事件/追踪 API **外部调用者为 0**。链路只是"活着"，没有业务行为。

| API | 协议命令 | 当前外部调用者 |
|---|---|---|
| `Vision_Link_RequestScanCode` | 0x10 | 0 |
| `Vision_Link_StartScan` | 0x28/0x38/0x48 | 0 |
| `Vision_Link_RequestMidline` | 0x20/0x30/0x40 | 0 |
| `Vision_Link_CancelMidline` | 本地取消 | 0 |
| `Vision_Link_RequestOrder` | 0x29/0x39/0x49 | 0 |
| `Vision_Link_TrackStart` | 0x22/0x25/0x32/0x42 | 0 |
| `Vision_Link_TrackStop` | 0x24/0x27/0x34/0x44 | 0 |
| `Vision_Link_ReportTcpPose` | 0x14 | 0 |
| `Vision_Link_GetTrack` | 0x23/0x26/0x33/0x43 | 0 |
| `Vision_Link_TakeEvent` | 事件 | 0 |
| `Vision_Link_IsOnline` / `GetStats` | 诊断 | 0 |

证据：`vision_link.h:127-160`、`vision_link.c:1107-1449`；生命周期仅 `app_init.c:22`、`app_tasks.c:89`。

### 按协议 §3 的推荐调用时机

| 时机 | 调用 | 驱动源 | 消费事件 |
|---|---|---|---|
| A 点到达 | `RequestScanCode()` 0x10 | **应有 A 的路径到达事件，目前缺失** | `SCAN_OK` / `SCAN_ERROR` / `SCAN_TIMEOUT` |
| B/C/D 扫描段开始前 | `StartScan(zone)` 0x28/0x38/0x48 | **应有扫描段开始事件，目前缺失** | 无应答 |
| 慢速扫描中 | `RequestMidline(zone)` 0x20/0x30/0x40 | 路径扫描段/定时器 | `MIDLINE`（1 成功，2/3 偏左/右） |
| 中位线成功后前进 | `RequestOrder(zone)` 0x29/0x39/0x49 | 路径前进/定时器 | `ORDER`（flags bit0=3 s 兜底） |
| 机械臂到预设位 | `TrackStart(target, 50)` | 机械臂完成轮询 | 无 |
| 追踪中每 5 ms | `GetTrack(target)` 取 dx/dy | 任务循环 | 无事件 |
| 任务完成/离开 | `TrackStop(target)` | 任务状态机 | 无 |
| TCP 位姿变化 ≤50 Hz | `ReportTcpPose(x,y,z)` 0x14 | 周期数据 | 无 |
| 每 5 ms | `TakeEvent()` 清空事件环 | 任务循环 | 所有事件 |

### 各区域职责与当前缺口

- **A 点扫码 0x10**：mission 在 A 到达时调用一次；`SCAN_ERROR` 的错误码 1..6 如何处置（重试/继续/中止）**业务规则未定义**。
- **B/C/D 开始扫描 0x28/0x38/0x48**：必须在慢速扫描段**开始前**调用；当前路径没有"扫描段开始"事件，只会在扫描结束后发 `SCAN_READY`，**mission 无法在正确时机调用**。需要新增 `PATH_EV_SCAN_BEGIN` 或暴露段类型。
- **中位线**：协议要求底盘运动时反复请求；当前扫描段开环直行，mission 只能在 `SCAN_READY`（扫描结束）拿到事件，且路径只保持 300 ms 就倒回。返回 2/3 需底盘左右微调，但 `chassis.h` **没有横向微调 API**。
- **顺序 0x29**：中位线成功后前进时请求；当前路径没有"前进到预设目标"的段，也没有按 order 选择预设位的机制。
- **追踪**：机械臂到预设位后启动；当前机械臂**没有完成事件**，只能轮询 `Arm_IsBusy()`/`Arm_GetLastError()`。
- **B 桶追踪 0x25→0x26→0x27**：B 物体放置完成后先停物体追踪，再启动桶追踪。
- **C 0x34/0x35**：停止追踪后开激光；`Arm_Laser` 无业务调用且 `ARM_LASER_ENABLE=0`。
- **D 0x44**：抓取/放置完成后停止追踪，再通知路径。
- **0x14**：需正解计算 TCP 位姿；`Arm_Kin_Forward()` 只在 `arm_kin.h:17` 内部可见，`arm.h` **没有 pose/关节角读取 API**，mission 无法计算。

### 事件驱动 vs 轮询

- 视觉事件驱动：扫码结果、中位线、顺序、目标丢失、5 次丢失、C 目标倒下、链路失联/恢复。
- 路径事件驱动：到达任务点、扫描完成、路径完成、路径错误。
- 机械臂轮询驱动：动作完成/失败（无事件）。
- 周期数据：0x14 位姿、GetTrack 读取。

### 视觉链路容量与预算证据

- 事件环 8（`vision_link_config.h:21-22`，实现 `vision_link.c:109`、`:564-586`、`:1413-1424`）。
- 追踪槽 4（`vision_link_config.h:23-24`，ISR 更新 `vision_link.c:377-390`，读取 `:1364-1410`）。
- RX 环 8（`vision_link_config.h:19-20`，入队 `vision_link.c:322-343`，出队 `:345-364`）。
- 每拍最多 4 帧、2 个 ACK、20 ms UART 超时（`vision_link_config.h:28-36`，`vision_link.c:543-560`、`:947-984`）。
- 中位线无总超时可无限重发（`vision_link.c:853-870`、`:1190-1196`）。
- 顺序 3 s 兜底（`vision_link.c:873-879`）；扫码 3 次超时（`:824-850`）。
- 失联只报事件并作废事务（`vision_link.c:649-703`）。
- 0x12 code5 静默清追踪（`:740-745`）；0x35 无条件事件（`:796-798`）。

## 4. 机械臂与底盘的编排缺口

- `Arm_Update()` 只由 MainTask 调用；动作完成**没有事件/回调**，上层只能轮询 `Arm_IsBusy()` / `Arm_GetLastError()`。
- `Arm_Act_Start*` 在反馈未就绪时返回 `ARM_BUSY`，调用方必须重试；只有 debug S18 实现了重试。
- 正式动作 `Home/Stow/MoveJoint/MovePose/PickAndPlace/Aim/Hold/Release/Laser` **无业务调用方**；仅 `Home/Stow/MoveJoint/MovePose` 被 debug 包装，`PickAndPlace/Aim/Hold/Release/Laser` 完全无调用。
- `Chassis_Path_NotifyTaskDone` 是唯一完成契约：单字节、无队列、无来源校验、无超时、无返回值；`point_id=0` 无法生效；任务等待无超时。
- 路径层**不检查控制层 fault**：`ctrl.h` 的 `fault` 只在内部可见，path 只判断 `finished/seq`。
- 失联/追踪丢失到达后**无人停臂、无人停车、无人回展开态**；"展开态"没有代码实现，只有 `Arm_Stow`（收拢）与 `Arm_Home`（q=0）。
- 机械臂与底盘之间**没有互锁**：路径移动时可同时启动机械臂；路径 DONE/ERROR 不自动收臂；底盘 fault 不停臂。

证据：
- `Arm_Update` 调用 `app_tasks.c:95-96`；内部 `arm.c:66-74`；状态/错误 `arm.c:84-104`；动作状态机 `arm_action.c:379-425`。
- 反馈未就绪返回 BUSY `arm_motion.c:116-121`；debug 重试 `app_debug.c:280-290`。
- 正式动作 `arm.c:106-151`；debug 包装 `arm.c:328-346`；唯一调用点 `app_debug.c:268-271`。
- `NotifyTaskDone` `path.c:524-527`，消费条件 `:496-504`，`point_id != 0` `:498`，Start 清空 `:381-382`。
- fault 不检查 `path.c:454-468`；`Chassis_Status_t` 含 fault `ctrl.h:41-54`；公开视图无 fault `chassis.h:72-93`。
- fault 只在 SPIN/ARC 检查 `ctrl.c:625-631`；fault 仍以 `finished=1` 上报 `ctrl.c:245-265`。
- 完成队列无人消费 `ctrl.c:262-265`、`:713-733`。
- 视觉事件只上报 `vision_link.c:649-703`、`:932-944`。
- 激光未启用 `arm_config.h:69-71`；`laser.c:25-38` 空操作；`Arm_Laser` 仍返回 OK `arm.c:149-151`。

## 5. 并发与实时性风险

### 5.1 单槽事件丢失场景

1. **双消费者**：debug 每拍清空，mission 看不到。
2. **陈旧通知**：`NotifyTaskDone` 在路径未等待时也锁存；下一任务点第一拍即放行，最后一段直接 DONE，ARRIVE 被覆盖。
3. **SCAN_READY 被 ERROR 覆盖**：倒回超时。
4. **Start 清空**：启动前事件丢失。
5. **跨任务撕裂**：`s_event` 三字段逐字段写/拷，无临界区；mission 若独立任务可能读到半新半旧值。

### 5.2 容量结论

- 事件环 8 本身够 5 ms 消费，但**当前无消费者，8 条后丢弃**，`LINK_LOST` 可能被丢。
- 追踪槽 4 够用；同一时刻只允许一个 target。
- RX 环 8 够用；追踪流绕过队列。
- 真正风险是事件环与路径单槽，不是 RX 环。
- 命令 API 绕过每拍预算，同一拍多次调用会叠加阻塞。

### 5.3 MainTask 阻塞与栈

| 调用 | 最坏阻塞 | 证据 |
|---|---:|---|
| `servo_send` | 约 22 ms | `servo_bus.c:213-225` |
| `step_send` | 约 20 ms | `step_uart.c:176-187` |
| S20 按键 1/3/7/8/9 | 约 22 ms | `app_debug.c:466-510` |
| S20 按键 2 | 约 20-22 ms | `app_debug.c:476-484`、`arm.c:227-230` |
| S18 动作启动 | 最坏约 42 ms | `arm_action.c:63`、`arm_motion.c:209-220` |
| `Arm_Fb_Update` | 一轮 4 轴约 150 ms | `joint_fb.c:92-135`、`servo_bus.c:329-364`、`step_uart.c:336-349` |
| `Vision_Link_Update` | 最坏 4×20=80 ms，正常远小于 1 ms | `vision_link_config.h:32-34`、`vision_link.c:543-560` |
| `App_Debug_HandleKey` | 直接违反非阻塞约束 | `app_debug.h:65-67`、`app_debug.c:466-510`、`:388-447` |

- MainTask 名义 5 ms，实际单拍可能阻塞 **20-100 ms**；CarTask/EncTask 优先级更高仍能运行，但路径、键盘、视觉 ACK/事件会被延迟。
- MainTask 栈仅 512 B；`Arm_Motion_StartJoint` 有 `q_now[4]/d[4]/pwm[3]` 等局部数组，加入 mission 后风险更高；建议至少 1 KB 并测 high-water mark。
- 栈溢出钩子直接 `bkpt` 停机（`freertos.c:60-70`），不可恢复。
- `Chassis_Ctrl_Update` PID dt 固定 5 ms（`ctrl.c:535`），CarTask slip 时积分/微分偏差。

### 5.4 周期宏

- 实际周期由 `KEYPAD_SCAN_PERIOD_MS = 5` 决定（`keypad.h:28`、`app_tasks.c:81`）。
- `CHASSIS_PATH_PERIOD_MS = 10` 定义但**全工程未使用**（`chassis_config.h:88`）。
- 修改键盘周期会无意改变路径节拍。

### 5.5 mission 放哪里

- 短期放 MainTask（与 `Vision_Link_Update` 同上下文），优先级保持 Normal。
- 不建议新建 Realtime/AboveNormal 任务；若单独建任务，必须把 `Vision_Link_Update` 一起移过去，并给路径事件、视觉事件环、Arm 状态加锁。
- 建议顺序：`Keypad → Debug → Vision_Update → Path_Update → Mission_Update → Arm_Update`。

## 6. 调试与运行入口

- 默认页固定 `S20`（`app_debug.c:23`）；唯一切页为长按 3 约 1.2 s 在 **S20 ↔ S18**（`:223-254`，条件 `:227`）。
- **S14 不可达**；S15/S16/S17/S19 也不可达。S14 按键代码存在（`:512-547`）但永不执行。
- 即使默认改 S14，也没有切页出口和中止键：注释写"9=中止"（`:514-515`），实际 9=启动整链（`:526-533`）。
- `CHASSIS_AUTO_RUN_PATH = 0`（`chassis_config.h:214-216`）；编译掉启动分支（`app_tasks.c:70-72`）。
- 即使改为 1，也只在初始化启动当前选中的表 0，**不会链式跑 A→B→C→D→终点**；链式逻辑只在 debug S14 按键 9。
- 没有全局底盘急停入口：`Chassis_Control_Stop`、`Chassis_Path_Abort` 均 0 调用者（`chassis.c:74-77`、`path.c:389-395`）；S18 的 3 只停机械臂。
- 当前现场按键：S20（1 释放扭矩 / 2 读位置 / 3 恢复扭矩 / 7/8/9 设置启动位置 / 长按 3 → S18）；S18（1 Home / 2 Stow / 3 停臂 / 7 测试关节 / 8 测试位姿 / 9 回读 / 长按 3 → S20）。**没有按键能进入 S14 或启动/中止底盘路径。**

## 7. 死代码 / 未接线

### 真正断链（业务不可用）

- 无 mission 模块/文件。
- 所有 Vision 命令/事件/追踪 API：0 外部调用者。
- A 点任务标记（`path_table.c:17-26`）。
- 扫描段开始事件不存在（`path.c:243-307` 只在扫描结束发 SCAN_READY）。
- 所有机械臂正式动作无业务调用者（`arm.c:106-151`）。
- 路径 fault 不传播（`path.c:454-468`）。
- 0x14 无调用者，且 app 层无正解/关节角 API（`arm_kin.h:17`、`arm.h:18-45`）。
- 视觉失联/目标丢失的停臂/停车/回展开无消费者。
- 激光未启用（`arm_config.h:71`），C 任务无法执行。

### 预留接口/代码（不一定错误）

- `Chassis_Path_SelectByName/Abort/IsRunning/IsWaitingTask` 均 0 调用者（`path.h:80-86`、`path.c:344-418`）；`IsRunning` 在 `PLAN_WAIT_TASK` 时返回 0，不能用于判断整条路径结束。
- `Chassis_Control_Stop` 0 调用者（`chassis.c:74-77`）。
- `Chassis_Ctrl_WaitDone` 0 调用者；完成队列 4 深无人消费（`ctrl.c:31-40`、`:262-265`、`:713-733`）。
- `PATH_TASK`、`PATH_STOP` 枚举存在但从未实例化（`path.h:27-28`）。
- `CHASSIS_PLAN_MAX_SEGS`、`CHASSIS_PLAN_SEG_GAP_MS` 定义未用（`chassis_config.h:210-212`）。
- `CHASSIS_PATH_PERIOD_MS` 定义未用（`chassis_config.h:88`）。
- `App_Debug_SetStep` 仅内部调用（`app_debug.c:113-116`、`:244-245`）。
- `s_last_path_ev` 只写不读（`app_debug.c:152`、`:167`）。
- `s_show_scan` 从未置 1（`app_debug.c:59`、`:471-503`），扫描页为死代码。
- `Arm_Debug_ServoScanStart/ServoPreset12/ServoQueryBroadcast/ServoScanIsDone` 0 调用者（`arm.c:284-311`、`arm_debug.h:65-75`）。
- `CHASSIS_AUTO_RUN_PATH` 分支被编译掉。

### TODO / 占位证据

- 调试模拟：`app_debug.h:50`、`app_debug.c:147-148`。
- 路径表占位：`path_table.c:9`。
- 底盘 TODO：`chassis_config.h:6`、`:13`、`:58`、`:191`。
- 机械臂 TODO：`arm_config.h:4`、`:16`、`:18`、`:29`、`:54`、`:109`；测试位姿 `:132`。
- 协议"回展开态"：`vision_link_config.h:53`、协议 §1/§6。
- 路径表注释与数值不一致：表 0 注释 355/80/390/300，实际 340/65/400/315（`path_table.c:15-25`）；表 1 注释扫描 600，实际 700（`:28-39`）；表 3 注释 360/500，实际 335/600（`:50-57`）。

## 8. 端到端可运行性判断

**按当前代码直接烧录：机器人会在起点静止，不会自动跑图。**

| 段 | 当前能完成 | 必然停住/缺失 |
|---|---|---|
| 起点 → A | 若被启动，底盘走完 7 段并发 DONE | **无 A 的 ARRIVE**；0x10 无人调用；mission 等在 A |
| A → B | 若被启动，CALIB + 直行/弧线 + 扫描 700 + SCAN_READY + 倒回 + ARRIVE | 无 mission；`SEG_TASK_WAIT` 无超时永久停；debug 500 ms 假完成放行但无视觉/机械臂任务 |
| B → C | 直行 1400 + 扫描 700 + SCAN_READY + 倒回 + ARRIVE | 同上；无 0x38/0x30/0x39/0x32/0x34/0x35；激光禁用 |
| C → D | 直行/弧线/直行 + 扫描 600 + ARRIVE | 同上；无 0x48/0x40/0x49/0x42/0x44、无抓取/放置 |
| D → 终点 | 直行 1700 + DONE | 无返回区任务/停车标记；需 debug 链或手动切表 |

四种场景：
1. 直接烧录：停在起点。
2. 改默认 S14 按 9：底盘能链式走完，但任务点全是 500 ms 假完成，视觉/机械臂不发生。
3. 接入 mission 但不修 A：停在 A。
4. 修 A 但不实现 B/C/D：停在 B 的 `SEG_TASK_WAIT`。

## 9. 逻辑断链清单（按阻塞程度）

| # | 断链 | 证据 | 类型 | 影响 |
|---|---|---|---|---|
| 1 | 无 mission 模块 | `src/app/`、`include/app/` 无文件 | 代码缺失 | 无法落地 |
| 2 | 路径事件唯一消费者是 debug，500 ms 假完成 | `app_tasks.c:93`、`app_debug.c:161-197` | 代码缺失 + 冲突 | mission 无法接管，提前放行 |
| 3 | A 点无 ARRIVE / 无 point_id | `path_table.c:17-26`、`path.c:121-134` | 代码缺失 | 卡在 A |
| 4 | 无扫描段开始事件；0x28/0x20/0x29 无调用时机 | `path.c:243-307`、`path_table.c:39/47/57` | 代码缺失 + 业务规则未定义 | 视觉流程无法启动 |
| 5 | 无路径入口：S14 不可达、AUTO=0 | `app_debug.c:23`、`:223-254`、`chassis_config.h:216` | 代码/配置 | 烧录不跑图 |
| 6 | 机械臂正式动作无业务调用方，无完成事件 | `arm.c:106-151`、`arm_action.c:379-425` | 代码缺失 | B/C/D 无法编排 |
| 7 | 路径不检查 fault，fault 当 finished | `path.c:454-468`、`ctrl.c:625-631` | 实现缺陷 | 陀螺掉线当成功继续 |
| 8 | 视觉失联/丢失无人停臂/停车/回展开；"展开态"未定义 | `vision_link.c:649-703`、`:932-944` | 代码缺失 + 业务规则 | 失联后继续运动 |
| 9 | 无按 order 选预设位/移到预设位的底盘 API；preset/tool_deg 缺失 | `chassis.h:32-39`、`arm_config.h` | 代码缺失 + 现场数据 | 0x2A 无法使用 |
| 10 | 无 dx/dy → 机械臂流式微调接口（PTP 最小 200 ms、BUSY） | `arm_config.h:125`、`arm_motion.h:6-7`、`arm_motion.c:106-108` | 代码缺失 | 50 Hz 追踪闭环无法实现 |
| 11 | 0x14 无调用者；app 层无 FK/关节角 API | `vision_link.h:151`、`arm_kin.h:17`、`arm.h:18-45` | 代码缺失 | 位姿无法上报 |
| 12 | 激光未启用、无 GPIO | `arm_config.h:69-71`、`laser.c:25-38` | 参数未标定 | C 段不可执行 |
| 13 | 单槽覆盖 + 陈旧 Notify + 等待无超时 | `path.c:48`、`:106-111`、`:496-504`、`:524-527` | 实现缺陷 | 事件丢/误放行/永久等待 |
| 14 | MainTask 单拍阻塞 20-100 ms；512 B 栈 | `servo_bus.c:213-225`、`step_uart.c:176-187`、`main.c:64-68` | 实时性 | 路径/视觉延迟，栈溢出风险 |
| 15 | debug 与 mission 无互斥；按键可关扭矩/停臂/启动路径 | `app_debug.c:466-510`、`:388-447` | 代码缺失 | 误操作/任务中断 |
| 16 | ctrl 队列 Put 返回值忽略；debug override 时 path 命令不被消费 | `path.c:212`、`ctrl.c:196-199`、`:543-547`、`:564-566` | 实现缺陷 | 路径超时/命令丢失 |
| 17 | fault 只在 SPIN/ARC 检查 | `ctrl.c:625-631` | 实现缺陷 | 直行陀螺掉线不报 |
| 18 | 视觉事件环无消费者会满；恢复后不自动重启事务 | `vision_link.c:564-573`、`:681-703` | 实现缺陷 + 业务规则 | 失联处理不可靠 |
| 19 | 扫描保持仅 300 ms 后自动倒回，视觉无握手 | `path_table.c:39/47/57`、`path.c:294-302` | 业务规则未定义 | 视觉来不及处理 |
| 20 | S14 注释"9=中止"与代码不符；无底盘急停 | `app_debug.c:514-515`、`:526-533` | 实现/文档 | 无法安全中止 |
| 21 | `CHASSIS_PATH_PERIOD_MS` 未使用 | `chassis_config.h:88`、`app_tasks.c:81` | 配置不一致 | 改节拍易踩坑 |
| 22 | `CHASSIS_PLAN_MAX_SEGS`/`SEG_GAP` 未用；ctrl dt 固定 | `chassis_config.h:210-212`、`ctrl.c:535` | 配置/实时性 | 参数误导、PID 偏差 |
| 23 | 死代码（s_last_path_ev/s_show_scan/Arm_Debug_ServoScan*） | `app_debug.c:59/152/933/1192`、`arm.c:284-311` | 死代码 | 维护成本 |
| 24 | 路径表距离/扫描长度占位，注释与数值不一致 | `path_table.c:9`、`:15-25`、`:28-40`、`:50-58` | 现场数据/参数 | 跑图位置不准 |
| 25 | OLED 无 mission 状态，S14 显示不可达 | `app_debug.c:23`、`:1275-1281`、`:1342-1357` | 调试缺失 | 现场无法观察 |

## 10. 一句话判断

视觉链路层已经"通电"，路径层已经"会走"，机械臂层已经"会动"，但三者之间没有任务大脑：没有 mission、没有视觉事件消费者、A 点没有到达事件、B/C/D 的任务完成仍由 app_debug 的 500 ms 假完成代替，机械臂正式动作没有业务调用方。当前烧录不会自动跑图；即使强行启动路径，也会在 A 点失去任务语义，并在 B/C/D 的 `SEG_TASK_WAIT` 永久停住（除非 debug 假完成）。

---

# 二、底盘与机械臂完整性（已完成）

> 构建数字说明：审查代理按链接器自然段（含 `._user_heap_stack`）统计为 RAM 28.3%、
> Flash 11.4%；PlatformIO 报告为 23.6% / 11.1%。两者口径不同，
> 需重新完整构建后用同一口径确认。

## 1. chassis_config.h：TODO、占位、单工作点与未使用宏

### 1.1 自带 TODO

- 文件头要求 TODO 必须填写（`chassis_config.h:6`）；机械/传动段 `TODO 实测`（`:13`）；速度/加速度段 `TODO 实测`（`:58`）；WIT 寄存器 `TODO 以实物手册核对`（`:191`）。

### 1.2 参数可信度

| 参数 | 值/位置 | 结论 |
|---|---|---|
| `MM_PER_PULSE` | 0.002242（`:45`） | 与理论/注释冲突，见 1.3 |
| 轮径/减速比/编码器 | 65 mm、30:1、1024×2（`:14-20`） | 仅用于理论回退；`PULSES_PER_REV=61440`（`:37`） |
| 编码器滤波 | ICF=0（`:23`） | 完全不滤波；噪声时建议 6~10，无实测 |
| 方向/极性 | L/R 宏（`:29-34`） | S4 实测 |
| 死区 | L=60 / R=150（`:55-56`） | 实测；左右差 2.5 倍，是航向不对称源 |
| 巡航速度 | 300（`:59`） | 路径表全部显式 200；仅 `seg_speed()` 回退值，实际未用；高于建议 200~250 |
| `V_MIN` | 40（`:60`） | 注释称"低于按停车"，实际只用于决定反向脉冲（`ctrl.c:280-282`） |
| 航向 KI/KD | 外 2.5 / 内 1.5 / KD 0.8（`:104-109`） | KI 仍整定；KD 必须保留 |
| 内环来源 | yaw 角度微分（`odom.c:128-143`） | 未用 0x52 角速度帧 |
| 圆弧补偿 | L=1.23 R=1.49 W=0.93 Kr=0.15（`:152-155`） | 仅 R200/200 mm/s 有效 |
| 圆弧最低速度 | 120（`:156`） | 会把反向圆弧负速度钳成正速度 |
| 提前制动 | 0.15 s / 10°（`:159-160`） | 用编码器角速度预测，打滑时不准 |
| 到点容差 | 5 mm / 0.5°（`:163-164`） | 无逐项实车记录 |
| 融合 alpha | 0.98（`:207`） | 编码器航向每拍 2% 注入 |
| 轮距 | 190（`:15`） | 无侧滑修正；`odom.c:82` 直接除 |
| 陀螺寄存器/速率 | `:191-200` | 寄存器 TODO；100 Hz；若配置失败降到 10 Hz，3 s 内 200 帧标定无法完成 |
| 自动跑图 | 0（`:216`） | 上电不自动运动 |

### 1.3 MM_PER_PULSE 三套口径冲突（重点）

- 理论：π×65/61440 = **0.003324 mm/pulse**。
- 代码：**0.002242**（`chassis_config.h:45`），比理论低 32.5%。
- `app_debug.c:651-665` 注释写"实测约 0.003，100000≈300 mm"。
- `DEBUG_NOTES.md:16` 写 0.002298，`:185` 又写 0.002242。
- 结论：轮径/减速比/编码器倍频/标定单位至少一项不一致。`USE_CAL=1` 时运行不出错，但距离模型不可解释。必须用 3000~5000 mm 复标并统一文档。

### 1.4 未使用/失效宏

- `CHASSIS_PATH_PERIOD_MS`（`:88`）：实际用 `KEYPAD_SCAN_PERIOD_MS=5`（`app_tasks.c:81`、`keypad.h:28`）。
- `CHASSIS_HEADING_WZ_FF`（`:114`）：`DEBUG_NOTES.md:68` 称已移除，但宏仍在且无引用。
- `CHASSIS_PID_DT_MIN_S`（`:99`）：PID 用自己的 `PID_DT_MIN_S`。
- `CHASSIS_PLAN_MAX_SEGS`（`:210`）、`CHASSIS_PLAN_SEG_GAP_MS`（`:212`）、`CHASSIS_CORRECT_MAX_TIMES`（`:180`）、`CHASSIS_GYRO_REG_SAVE/REG_BAUD/REG_YAW_ZERO`（`:192-196`）均无引用。
- `CHASSIS_V_CRUISE_MMPS` 仅作回退值；`CHASSIS_ENC_FILTER_ICF=0` 使滤波函数不写寄存器（`encoder.c:35-44`）。

## 2. path_table.c：五张表

### 2.1 占位声明

文件头明确"距离、扫描段长度、任务超时均为占位值"（`path_table.c:9`）。

### 2.2 注释与实际对照

| 表 | 注释值 | 实际值 | 结论 |
|---|---|---|---|
| 起点→A | 355/80/390/300（`:15-16`） | 340/65/400/315（`:19-25`） | 四处不一致 |
| A→B | 1350/380/扫描 600（`:29`） | 1350/385/扫描 700（`:34-39`） | 直行差 5，扫描差 100 |
| B→C | 1400/扫描 700/倒回 350（`:43`） | 1400/700（`:46-47`） | 倒回由扫描距离一半计算，不是固定 350 |
| C→D | 360/300/扫描 500（`:51`） | 335/300/600（`:54-57`） | 直行差 25，扫描差 100 |
| D→终点 | 1700（`:61`） | 1700（`:64`） | 一致但仍是占位 |

### 2.3 dwell、timeout、其他字段

- `dwell_ms` 只有扫描段 300 ms（`:39,47,57`）；无 `PATH_STOP` 段。
- 所有 `timeout_ms=0`，走自动估算（`:19-64`、`path.c:60-104,215-216`）。
- 所有 `a_mmps2=0`，且 `Path_Seg_t.a_mmps2` 全工程无读取（`path.h:40`）。
- `brake=1` 全部段。
- `point_id` 仅 B/C/D 扫描段；表 0 无点号或 `PATH_TASK`（`:17-26`）。

### 2.4 自动超时估算

`seg_estimate_ms()`：`dist/v×1000×3`，最小 2 s；圆弧按弧长；旋转按 75°/s；STOP 按 dwell×3（`path.c:60-104`）。

| 段 | 估算 |
|---|---|
| 直行 340 @200 | 5.1 s |
| 直行 65 @200 | 2.0 s（最小钳位） |
| 直行 400 @200 | 6.0 s |
| 直行 315 @200 | 4.725 s |
| 圆弧 90°/R200 @200 | 4.71 s |
| 扫描 700 @100 | **21 s** |
| B→C 直行 1400 @200 | **21 s** |
| C→D 扫描 600 @100 | **18 s** |
| D→终点 1700 @200 | 25.5 s |
| `PATH_CALIB` | 固定 5 s（`path.c:259-261`） |

`[新发现]` 扫描倒回复用前进扫描段的估算（`path.c:238-239`）：倒回 350 mm @200 只需约 1.75 s，但超时给 18~21 s；控制层失控要等十几秒才报错。

### 2.5 A 点为什么无 PATH_TASK

- 表 0 最后一段是普通直行，无 `point_id`（`path_table.c:25`）。
- 只有 `PATH_TASK` 段（`path.c:264-268`）和扫描倒回完成（`:457-462`）产生 ARRIVE。
- 表 0 两者都没有，走完直接 DONE（`:121-131`）；debug 收到 DONE 后把链式索引推进到 A→B，A 任务被跳过（`app_debug.c:175-183`）。

### 2.6 300 ms 扫描停留是否够

- 扫描走完先发 `SCAN_READY`，等 300 ms 再倒回（`path.c:285-303`）。
- 扫码重发周期 1000 ms、中位线 500 ms（`vision_link_config.h:40-45`）。
- 300 ms **不够一次完整请求-应答**，最多够触发或发一条命令。
- 若 mission 在 `SCAN_READY` 取图，倒回期间相机在动；若在 `ARRIVE` 取图，300 ms 只是延迟。
- 取图时机必须由业务规则明确。

## 3. path.c：状态机、超时、倒回、事件、握手

### 3.1 段状态机

`SEG_IDLE`（`path.c:441-443`）、`SEG_ACTIVE/SEG_SCAN_BACK`（`:445-469`）、`SEG_DWELL`（`:471-476`）、`SEG_SCAN_HOLD`（`:478-483`）、`SEG_CALIB`（`:485-494`）、`SEG_TASK_WAIT`（`:496-504`）。

### 3.2 超时

- ACTIVE/SCAN_BACK 有超时（`:447-451`），超时 `path_error()`。
- CALIB 5 s（`:259-261,490-493`）；DWELL 用 dwell（`:251,471-475`）。
- SCAN_HOLD 无超时（`:478-483`）。
- **TASK_WAIT 无超时**（`:496-504`）——任务链永久卡死的直接风险。
- `path_error()` 调 `Chassis_Ctrl_Stop(1)` 置 `PLAN_ERROR`（`:113-119`），不记录原因。

### 3.3 倒回中心

- `s_scan_dist = st.dist_done_mm`（`:289-292`）；倒回一半（`:220-241`）。
- `dist_done_mm` 是编码器轮距，打滑时偏大（圆弧 DST 偏大 30%~40%）。
- "倒回一半"只对编码器距离成立，不保证几何中心。
- 倒回命令清速度环积分（`:236`），方向处理正确。

### 3.4 事件单槽

- 单槽（`path.c:48-49`），`path_emit()` 覆盖（`:106-111`），`TakeEvent()` 取后清空（`:512-522`）。
- `SCAN_READY` 未取走会被 `ARRIVE_POINT` 覆盖。
- 两类事件使用同一个 `point_id`，mission 必须区分，不能重复执行任务。

### 3.5 NotifyTaskDone 契约

- 只写一个字节（`:524-527`）；`TASK_WAIT` 只有 `s_task_done_id != 0 && == seg->point_id` 才推进（`:498-503`）。
- 风险：错误点号/丢通知 → 永久等待；提前通知 → 立即放行；`NotifyTaskDone(0)` 被拒绝；连续通知只留最后一个。
- 唯一调用者是 500 ms 调试模拟（`app_debug.c:191-196`、`app_debug.h:51`）。

### 3.6 fault

**不检查。**

- 控制层故障也以 `finished=1` 上报（`ctrl.c:245-265`、`:626-631`）。
- 路径只检查 `finished && seq`（`path.c:454-468`），不看 `fault`。
- `Chassis_Ctrl_WaitDone()` 能返回 FAULT，但全工程无调用者（`ctrl.c:713-733`）。

### 3.7 命令/完成握手竞态

- **队列满**：`Submit()` 先累加计划航向（`ctrl.c:185-186`），再无阻塞入队并忽略返回值（`:196-199`）；路径层已设 `s_seg_seq`（`path.c:209-212`）。队列满时命令未执行、航向已偏、路径只能超时。
- **stale finished/seq 复用**：`ctrl_finish()` 置 `finished=1`（`ctrl.c:245-260`）；空闲分支不清（`:618-623`）；`Chassis_Ctrl_Stop()` 也不清（`:863-877`）；`Path_Start()` 把 seq 清零（`path.c:376-384`）。若上条路径末段 seq 与新路径首段相同，可能立即读到残留 `finished=1` 误判完成。CarTask 是 Realtime、概率不高，但协议没有"命令已接收"确认。
- **完成队列**：`s_q_done` 长度 4，满后静默丢弃（`ctrl.c:262-265`）；`WaitDone` 无调用者，队列永不消费。

### 3.8 路径中止

- `Chassis_Path_Abort()` 无调用者（`path.c:389-395`）。
- `app_debug.c:515` 注释写"9=中止"，实际 9=启动整链（`:526-533`）。
- `Chassis_Control_Stop()` 无 app 调用者（`chassis.c:74-77`）。
- 当前没有运行时中止入口，注释与实现不一致。

### 3.9 节拍

- `path.h:9`、`path.c:6` 写"每 5 ms"；`CHASSIS_PATH_PERIOD_MS=10` 未使用；实际由 `KEYPAD_SCAN_PERIOD_MS=5` 推进（`app_tasks.c:81`、`keypad.h:28`）。

## 4. ctrl.c

### 4.1 Submit 忽略队列满

- `s_planned_heading` 无回滚（`:185-186`）；`s_path_anchored==0` 时自动 `BeginPath()`（`:180-183`）。
- `Chassis_Ctrl_Stop()` 不清 `s_path_anchored` 和 `s_planned_heading`（`:863-877`），停止后再提交不会重新锚定。

### 4.2 固定 dt 与实测 dt

- 控制层固定 5 ms（`ctrl.c:535`）；感知层实测 `HAL_GetTick()`（`odom.c:66-73`）。
- `pid.h:8` 要求调用方传实测周期，与控制层矛盾。

### 4.3 航向积分跨段保持

- `s_pid_heading` 只在模式变化时复位（`ctrl.c:591-595`）；直行段之间、停车期间保留。
- `Chassis_Ctrl_Stop()` 不复位航向 PID、不更新 `s_last_mode`（`:863-877`）。
- 停车/被推/重新摆位后，残留积分会让下次起步带方向偏置。

### 4.4 end_speed 没有实现真正的段间平滑（重点）

- 路径层给非末段计算下一段入口速度（`path.c:136-166`）并写入 `cmd.end_speed_mmps`（`:206`）。
- 直行只用它影响减速曲线（`ctrl.c:355-360`）；剩余 ≤5 mm 时无条件 `ctrl_arrival()`（`:392-395`）。
- `ctrl_arrival()` 设置 `s_arrived` 并可选反向脉冲（`:275-287`）；下一拍 60 ms 后 `ctrl_finish()`（`:655-667`）；`ctrl_finish()` 无条件 `Motor_StopAll(1)`（`:271-272`）。
- 后果：中间段即使 `end_speed>0` 仍反向脉冲 + 短路制动 + 停车。以 340 mm 直行接圆弧为例，`end_v=200`，剩 5 mm 时允许约 206 mm/s，实际以约 200 mm/s 到段末然后反向脉冲，**比普通减速停车更粗暴**。
- 圆弧同理，`end_speed` 只用于 `v_allow`（`:478-480`）。
- `DEBUG_NOTES.md:187` 的"段间平滑已实现"不成立；`cmd.seg_last` 在控制层从未使用（`ctrl.h:37`）。
- 真要连续过段，控制层必须在 `end_speed>0` 时不进入 `ctrl_arrival()/ctrl_finish()`。

### 4.5 旋转/圆弧/制动未覆盖分支

- 旋转：减速（`:409-414`）、最小角速度（`:416-422`）、角加速度限制（`:424-434`）、到点（`:439-442`）；每次进入 SPIN 复位 PID 与 `s_spin_w_prev`（`:585-589`）；误差无 `Wrap180`（`:407`）。
- 圆弧：半径 <1 走退化分支（`:464-469`、`kin.c:60-65`），无减速和径向反馈；**反向圆弧被最小速度钳成正速度**（`:485-488`）；末端预测用编码器角速度（`:517`），打滑时不准。
- 制动：参数按模式（`:290-310`）；脉冲幅值钳位（`:326-328`）；结束短路制动（`:646-651`）；到位等 60 ms（`:655-667`）；单轮为 0 时按 `>=0` 反向可能转向（`:330-331`）；进入制动要求至少一轮 >40（`:280-282`）。
- 停止命令：`CHASSIS_MODE_STOP` 基本未使用（`:687-690`）。

### 4.6 调试命令未初始化（新发现）

三个调试函数的 `Chassis_Cmd_t cmd` 未初始化、无 `memset`、未设 `end_speed_mmps`：

- 直行 `ctrl.c:787-810`、圆弧 `:817-838`、旋转 `:840-861`。
- 控制层读取该字段（`:355-360,478-480`）；路径层有 `memset`（`path.c:174,226`）。
- 影响：S10/S11/S12 末端减速依赖栈垃圾值。

### 4.7 调试接管与队列残留

- `Chassis_Ctrl_DebugRunSpeed()` 只置标志，不清队列（`:759-773`）；`SetDebugOverride(1)` 不清队列、不主动停车（`:747-757`）。
- S5 脉冲调试进入 `s_pulse_hold` 后每 5 ms 重复制动、`s_debug_override` 保持，直到按 7 `StopPulses()`（`chassis.c:152-196`）；忘记释放后正式运动被 `ctrl.c:543-547` 直接跳过。

### 4.8 完成状态残留

- `ctrl_finish()` 置 `s_telem.finished=1`（`:245-260`）；空闲分支与 `Ctrl_Stop()` 都不清（`:618-623,863-877`）；路径每次 Start 复位 seq（`path.c:376-384`）。
- 构成 3.7 的潜在误判；严格修复应在提交新命令时清 `finished` 或返回"已接收"。

### 4.9 其他

- `CHASSIS_V_MIN_MMPS` 注释与实际用途不符（`chassis_config.h:60`、`ctrl.c:280-282`）。
- `ctrl_run_straight()` 对 `dist_mm=0` 立即完成，而 `ctrl.h:34` 注释说"<=0 表示不定长"，接口语义不一致（`ctrl.c:340-341,392`）。
- `ctrl_run_arc()` 的 `s_arc_valid==0` 分支无径向反馈和减速（`:500-513`）。

## 5. odom.c

### 5.1 融合轮距 190 未修正侧滑

- `dth=(mm_r-mm_l)/190`（`odom.c:81-82`）；`yaw += 0.98*dyaw + 0.02*dth`（`:147-156`）。
- 履带转向侧滑使编码器航向高估转角，2% 权重缓慢注入；F_test_VE 用 218.48 修正值。

### 5.2 角速度来源

- yaw 微分 + 20 ms 低通（`:128-143`）；未用已解析的 0x52；`Chassis_Gyro_GetWzDps()` 只用于调试显示（`chassis.c:215`）。
- 编码器差速角速度 `w_degps` 仍输出（`:108`），用于圆弧提前制动（`ctrl.c:517`）。

### 5.3 陀螺掉线与标定

- 掉线时融合权重切 0，完全依赖编码器（`:146-156`）；直行不检查陀螺在线，只有 SPIN/ARC 报 fault（`ctrl.c:626-631`）。
- `Chassis_Calibrate()` 返回值被忽略（`app_tasks.c:68`）；3 s 内没收到 200 帧时零偏保持 0，控制继续。
- 配置寄存器 TODO，自动配置只写不保存（`gyro.c:214-220`）。

### 5.4 MarkSegStart 与段位移

- `Chassis_Odom_MarkSegStart()` 无调用者（`odom.c:209-215`）；`ds_seg_mm` 正常运行从不清零，控制层用 `s_dist_start` 快照（`ctrl.c:577-580`）；`yaw_seg_deg` 只靠 Reset 清零（`odom.c:193-207`）。
- 不是错误，但 `MarkSegStart` 是死接口。

### 5.5 长跑三角函数风险

- 无界 yaw 直接送入 `sinf/cosf`（`:159-161`）。
- `[推断]` `|yaw|` 超过约 200 rad（约 32 圈）时 libm 大参数规约栈帧约 372 B；EncTask 512 B 有溢出风险。常规比赛可能达不到，连续旋转测试需注意。

## 6. 底盘能力清单

### 已实现

电机/编码器方向与距离标定、左右独立速度环+前馈、航向串级、原地旋转、R200 圆弧、反向脉冲+短路制动、非阻塞路径状态机、五张路径表、扫描段+倒回一半、路径事件与通知、陀螺解析/校验/掉线/自动改速率、OLED/键盘与路径并行（证据见各节）。

### 已实现但未整定

航向 KI、300 mm/s 高速直行、圆弧补偿、路径表距离、段间平滑 `end_speed`（实际仍停车）、到点容差/制动脉冲。

### 未实现

mission、视觉联调、A 点事件、fault 传播、任务等待超时、路径中止入口、多事件队列、队列满回滚/命令确认、S14 入口、自动跑图、栈高水位监测。

## 7. chassis.h 中 app 未调用的接口

| 接口 | 调用情况 |
|---|---|
| `Chassis_Control_Stop()` | 零调用者（`chassis.c:74` 仅定义） |
| `Chassis_CalibrateStart/Poll()` | 仅 `path.c:258,486` 内部调用 |
| `Chassis_Debug_ResetPulses()` | 仅 `chassis.c:154` 内部 |
| `Chassis_Path_SelectByName()` | 零调用者 |
| `Chassis_Path_Abort()` | 零调用者 |
| `Chassis_Path_IsRunning()` | 零调用者 |
| `Chassis_Path_IsWaitingTask()` | 零调用者 |

有 app 调用但只在 S14 分支：`SelectById/Start`（`app_debug.c:531-542`）、`GetState`（`:1346`）。当前 S3~S14 全部无运行时入口，不只是 S14。

## 8. arm_config.h

### 8.1 实测/已配置

| 参数 | 值 | 判断 |
|---|---|---|
| J2/J3/J4 零位 | 846/858/1559（`:39-41`） | 注释实测，可信度较高 |
| 方向 | 均 +1（`:42-44`） | 需整机复核 |
| 限位 | Q2 0~182、Q3 0~179、Q4 -89~79（`:99-106`） | 注释实测，约 1° 余量 |
| 连杆 | H0=55、大臂 120、偏置 20、小臂 160、TCP(80,55)（`:80-86`） | 注释实测，偏置方向/TCP 未验证 |
| 舵机 ID | 1/2/3（`:29-31`） | 已配置，行内仍 TODO |
| 减速比 | 7:1（`:17`） | 未见实测 |
| 夹爪 | 45/105/135°（`:58-66`） | 基于 270° 假设 |

### 8.2 假设/占位

| 参数 | 位置 | 问题 |
|---|---|---|
| 步进细分 | 3200（`:16`） | 明确 TODO |
| 步进速度/加速度 | 30 RPM / 档位 10（`:18-19`） | 明确 TODO |
| 半双工极性 | TX=1 / RX=1（`:54-56`） | 明确 TODO |
| 舵机角度范围 | 270°（`:35`） | 假设；若 180° 全部换算错误 |
| 夹爪 270° | `:58-66` | 若 180° 需重算 |
| 激光 | `ENABLE=0`（`:71`） | 引脚未分配，空操作 |
| 工作空间 | r 30~400、z -100~300（`:108-113`） | TODO；与 `ARM_DEBUG_NOTES.md:95` 的 z≈315 冲突 |
| 收拢姿态 | (0,181,178,78)（`:115-120`） | 限位推导，未整机验证；接近上限仅 1° |
| PTP 参数 | 90/120、0.2 s、2°、1500 ms、3 次（`:122-130`） | 全设计值；`STEP_W_MAX` 未使用 |
| 测试位姿 | (150,0,120,0)（`:132-136`） | 调试用，不是任务位姿 |
| 任务 tool_deg | `arm_types.h:39` 字段存在；B/C/D 未提供 | 阻塞业务 |

### 8.3 未使用宏

`ARM_STEP_ZERO_DEG/DIR_SIGN`（`:23-24`）、`ARM_UPPER_AXIS_MM`（`:83`）、`ARM_PTP_STEP_W_MAX_DEGPS`（`:124`）、`ARM_FB_POLL_MS`（`:145`）、`ARM_GRIPPER_MOVE_MS`（`:67`）、`ARM_DEBUG_JOG_DEG/SERVO_JOG_DEG`（`:149-150`）。`ARM_DEBUG_SYNC_MOVE_MS` 只在不被调用的 `Arm_Debug_ServoPreset12()`（`arm.c:284-290`）。

## 9. arm_kin.c

### 9.1 坐标系与公式

- 文件注释：x 车体正前、z 向上；θ1 绕 z；俯仰面 (r,z)；θ2 轴在 (0,H0)；大臂 120，末端垂直偏置 20 到 θ3；小臂 160；TCP 局部 (80,55)；q3=0 时小臂与大臂同向（`arm_kin.c:5-10`）。
- 正解：`r=A cos q2 - B·sign·sin q2 + C cos(q2+q3) + F cos φ - U sin φ`；`z=H0 + A sin q2 + B·sign·cos q2 + C sin(q2+q3) + F sin φ + U cos φ`；`φ=q2+q3+q4`（`:41-54`）。
- 逆解：先扣 TCP 偏移（`:96-102`），余弦定理两分支（`:104-132`），按 `ARM_IK_ELBOW` 选 q3 符号（`:134-140`），`q4=φ-q2-q3`，`q1=atan2(y,x)`（`:94`）。

### 9.2 20 mm 偏置符号

`ARM_ELBOW_OFFSET_SIGN=+1`（`arm_config.h:89`）。+1 时 r 增加 `-B sin q2`、z 增加 `+B cos q2`；q2=0 时 z +20。零位 TCP r=360、z=130，与 `ARM_DEBUG_NOTES.md:93` 一致。未实机验证。

### 9.3 TCP 是否随 θ4 旋转

**代码假设随 θ4 旋转**：正解用 `φ=q2+q3+q4` 与 TCP (F,U)（`:41-54`）；逆解用 `tool_deg` 扣偏移（`:96-102`）；FK 输出 `tool_deg=q2+q3+q4`（`:59`）。未实机验证。

### 9.4 肘部构型

`ARM_IK_ELBOW=+1`（`arm_config.h:92`）；`pick_a=((q3_a>=0)==(ELBOW>=0))`（`:134-136`），+1 选 q3>=0（肘上）。若设反，q3 落负角而 Q3 限位 0~179°，所有目标被限位拒绝。

### 9.5 未实机验证的假设

偏置符号、TCP (80,55) 及是否随 q4 旋转、q3=0 定义、肘上/肘下、H0/连杆长度。`Arm_Kin_Forward()` 全工程无调用者（`:22`），没有运行时 FK 校验或 FK/IK 互查。

### 9.6 其他

`ARM_UPPER_AXIS_MM=121.66` 未使用（`arm_config.h:83`）；逆解对 `rm<KIN_EPS` 与 ratio 越界有保护（`:112-123`）；q1 来自 atan2，覆盖 Q1 限位；`tool_deg` 只影响俯仰面。

## 10. arm_action.c

### 10.1 状态机与完成条件

- 状态 `ACT_IDLE/BUSY/DONE/FAULT`（`arm_action.c:16-22`）；状态/错误/超时/步骤为 static（`:24-35`）。
- `act_begin()`：已 BUSY 返回 `ARM_BUSY`（`:53-56`）；关节限位（`:58-61`）；`Arm_Motion_StartJoint()`（`:63-67`）；成功后置 BUSY、记录超时（`:69-77`）。
- `act_motion_done()`：先查运动层 fault（`:80-86`），再查 `IsDone`（`:88`）。

### 10.2 Home / Stow / Joint / Pose

- 均为"一次 PTP + 等待完成"：Home q=0（`:235-240`）、Stow 用配置姿态（`:242-252`）、Joint 直接输入（`:254-257`）、Pose 先 IK 再限位（`:259-279`）。
- 完成判定 `act_motion_done()>0`（`:406-414`）；动作级超时（`:386-390`）。
- 问题：Pose/Aim 只用 `Arm_Limit_CheckJoints()`，**不调用 `Arm_Limit_CheckPose()`**（`:268-277`、`:340-350`）；工作空间只在 Pick&Place 使用（`:304-307`）。
- Home 的 q=0 是"大臂水平、小臂伸直、腕部零位"，不是收拢姿态；零位未标定时可能撞限位（`ARM_DEBUG_NOTES.md:163-165`）。

### 10.3 Pick&Place 的 16 步

步骤：OPEN → pick_above → pick → GRASP → pick_above → place_above → place → OPEN → place_above → DONE（`arm_action.c:108-223`）。

问题：

- **夹爪没有任何等待时间**：OPEN/GRASP/OPEN 后立即进入下一步（`:114-115,149-152,199-202`）；`ARM_GRIPPER_MOVE_MS=800` 未使用（`arm_config.h:67`）。会出现"还没夹住就抬臂/还没松开就抬臂"。
- **没有每步超时**，只有动作级默认 8 s（`:325-326`、`arm_config.h:153`）；16 步含 4 次运动，容易超时误报。
- 依赖未标定 IK/姿态：`pick_above/place_above` 只是 z 加 `approach_mm`（`:299-302`）；4 个位姿都必须 IK + 限位通过（`:304-318`）；`tool_deg` 未提供。
- `approach_mm` 无正负检查，负值会把"上方"变"下方"，仍可能通过 workspace。
- 完成后不检查夹爪是否真的夹住（无回读）。

### 10.4 Aim

- `StartAim()`：IK + 关节限位 + `act_begin()`（`:331-351`）；完成时开激光并置 DONE（`:398-403`）。
- 谁负责关：`act_fault()`（`:37-45`）、`Arm_Act_Abort()/Arm_Stop()`（`:437-447`、`arm.c:76-82`）；**没有定时关闭、没有目标倒下自动关闭、没有启动下一动作时关闭**。Aim 成功后激光会一直开着，直到 mission 显式关闭；当前没有 mission。
- **`ARM_LASER_ENABLE=0` 时 Aim 仍报告成功**：`Arm_Actuator_Laser()` 无论可用性都返回 0（`arm_actuator.c:48-53`）；`Arm_Laser()` 返回 `ARM_OK`（`arm.c:149-152`）。

### 10.5 BUSY 重试与错误上报

- `act_begin()` 返回 BUSY 时**不建立动作状态和超时**（`:53-56`）；调试页每 50 ms 重试、2 s 超时（`app_debug.c:256-298`）。
- 正式 mission 若只调一次收到 BUSY，必须自己重试；此时 `Arm_IsBusy()` 也为 false（`arm.c:84-87`），容易误判"没有动作"。
- `act_fault()` 停运动、夹爪 HOLD、关激光、置 FAULT（`:37-45`）；`Arm_GetLastError()` 优先 ACT 再 Motion（`arm.c:89-104`）。
- **`Arm_Stop()` 把 `s_error` 清成 `ARM_OK`**（`arm_action.c:437-446`），急停与故障停机不可区分，无故障锁存。

### 10.6 Stow 依赖

- `ARM_STOW_Q1=0` 未实机确认；q2=181、q3=178、q4=78 都接近上限，只有约 1° 余量；齿隙/回读误差可能触发限位拒绝（`arm_config.h:115-120`）。

## 11. arm_motion / arm_limit / arm_actuator / joint_fb

### 11.1 到位容差与回读有效性

`Arm_Motion_StartJoint()`：

- 要求四轴掩码 `0x0F`，否则发起回读并返回 BUSY（`arm_motion.c:116-121`）。
- 用回读角计算增量（`:123-127`）；四轴都 <0.05° 直接 DONE（`:129-141`）。
- 舵机时间按 90°/s 取最大；J1 RPM 限 2~120（`:143-181`）；运动时间 >9 s 返回 PARAM（`:183-186`）。
- PWM 越界返回 LIMIT（`:190-203`）；下发同步帧与步进位置（`:205-220`）；立即 `Arm_Fb_AddJ1Delta()`（`:222-225`）。

完成判定：

- 等运动时间过去（`:263-267`）；每 250 ms 刷回读（`:269-274`）。
- 四轴掩码必须 `0x0F`（`:276-279`）；每轴误差 ≤2°（`:281-295`）；连续 3 次满足才 DONE（`:297-307`）。

问题：

- **"稳定 3 次"实际是 3 个 5 ms 控制拍**，不是 3 次新回读；两次回读之间返回同一缓存（`joint_fb.c:137-178`）。一次回读落进容差后 15 ms 就 DONE。
- **完成判定不检查回读时间戳**：舵机 online 允许最近 1 s 内有过应答（`servo_bus.c:639-650`）；`GetJointsMask()` 只要曾收到过且 online 就返回旧 PWM（`joint_fb.c:163-174`）。舵机掉线/卡住时，旧目标值可能在 1 秒内继续被当作有效，误判到位。
- **J1 完全没有真实回读参与判定**：命令下发时 `Arm_Fb_AddJ1Delta()` 就把软件位置改到目标（`arm_motion.c:222-225`）；`GetJointsMask()` 直接返回 `s_j1_pos_deg`（`joint_fb.c:156-160`）；完成判定里 J1 永远"已在目标"，无法发现丢步、堵转、复位。`Arm_Step_GetPositionDeg()` 存在但无调用者（`step_uart.c:352-369`）。
- PTP 启动也用可能陈旧的 J1 软件位置；上电后手动转动 J1 不会更新软件位置，运动时间计算会错。
- **`s_servo_target[]` 只由调试点动更新**（`arm.c:24,59-61,192-224`）；正式 PTP 不更新，下一次 S16 点动会从启动零位而非当前位置开始，可能大幅跳动。

### 11.2 限位与工作空间

- `CheckJoints()`：Q1 ±180、Q2 0~182、Q3 0~179、Q4 -89~79（`arm_limit.c:20-35`）；只比较，NaN 全部通过。
- `CheckPose()`：r 30~400、z -100~300（`:51-58`），再 IK + 关节限位（`:60-65`）。
- Pose/Aim 不调用 `CheckPose()`；NaN 输入会通过限位，随后 `Wrap180(NaN)`、`lroundf(NaN)`、PWM 计算属于未定义行为。
- `ARM_Z_MAX=300` 与 `ARM_DEBUG_NOTES.md:95` 的 z≈315 冲突；当前 MoveTestJoint 绕过 workspace，未来用 Pose 会被拒绝。

### 11.3 离线判定

- 步进：最近 1 s 内收到任意合法帧即在线（`step_uart.c:399-403`）。
- 舵机：最近 1 s 内收到 PRAD 即在线（`servo_bus.c:639-650`）。
- 运动层没有"离线即 fault"，只依赖掩码与超时：掩码不完整等待（`arm_motion.c:276-279`），超时 fault（`:255-261`）。最坏等运动时间 +1500 ms 才报 TIMEOUT。
- 步进单独复位时软件位置不归零；只要还能应答，`Arm_Step_IsOnline()` 仍真，J1 沿用旧累计值。**代码没有实现复位识别**，只做超时判离线。

### 11.4 会导致卡死或误判的路径

| 路径 | 问题 |
|---|---|
| 反馈未就绪 | `StartJoint()` 返回 BUSY，`act_begin()` 不置 ACT_BUSY、不启动超时（`arm_action.c:53-67`）；调用方不重试就永远不开始 |
| 舵机掉线/卡住 | 旧回读 1 s 内仍有效（`joint_fb.c:163-174`、`servo_bus.c:639-650`），可能误判到位 |
| J1 丢步 | 软件位置命令时已到目标（`arm_motion.c:222-225`、`joint_fb.c:156-160`），永远判到位 |
| 稳定判定 | 3 个控制拍而非 3 次回读（`arm_motion.c:297-307`） |
| Pick&Place | 无每步超时、默认 8 s；夹爪无等待（`arm_action.c:114-115,149-152,199-202,325-326`） |
| 动作超时 | 全局超时后 `act_fault()` abort，不会永久卡死，但会丢任务（`:386-390`） |
| 激光不可用 | Aim 仍成功（`arm_actuator.c:48-53`、`arm_action.c:398-403`） |
| 急停 | `Arm_Stop()` 清错误，无法区分故障与主动停止（`arm_action.c:437-446`） |

## 12. 激光/夹爪

### 12.1 laser.c 在 ENABLE=0 的行为

- `Init()` 只清 `s_on`（`laser.c:21-28`）；`Set()` 只更新软件状态（`:30-38`）；`IsOn()` 返回软件状态（`:40-43`）；`IsAvailable()` 返回 0（`:45-48`）。
- `ENABLE=1` 但未定义端口/引脚会编译报错（`:11-17`）。
- 后果：S17 显示 `LASER N/A`（`app_debug.c:1050-1060`）；`Arm_Actuator_Laser()` 总返回 0（`arm_actuator.c:48-53`）；Aim 报告成功。

### 12.2 夹爪时序与无回读

- `Init()` 先写 1500 µs 再启动 PWM（`gripper.c:16-23`）；`SetPulseUs()` 直接改比较值，无延时、无回读（`:25-38`）。
- `ARM_GRIPPER_MOVE_MS=800` 未被使用（`arm_config.h:67`）。
- `Arm_Actuator_Gripper(HOLD)` 不发命令，只改软件状态（`arm_actuator.c:37-39`）；调试页把 HOLD 显示成"开"，因为 `gripper_closed` 只认 CLOSE/GRASP（`arm.c:412-414`）。
- 夹爪角度基于 270° 假设（`arm_config.h:58-66`）。

### 12.3 谁调用激光/夹爪

- 正式：夹爪 `arm_action.c:114-115,149-152,199-202,353-377`、`arm.c:139-147`；激光 `arm_action.c:37-45,398-403`、`arm.c:149-152`。
- 调试：`Arm_Debug_Gripper/Laser`（`arm.c:232-237,323-326`）、S17 按键（`app_debug.c:371-386`）。
- 当前 S17 不可达；夹爪键只调 OPEN/CLOSE，不调 GRASP；激光键在 ENABLE=0 时无输出。HANDOVER "夹爪测试键未加"实际是"有 S17 但不可达且不含 GRASP"。

## 13. arm.h 未被 app 调用与 Arm_Update

### 13.1 未接入接口

| 接口 | 调用情况 |
|---|---|
| `Arm_PickAndPlace()` | 零调用者；ELF 中也无 `Arm_Act_StartPickPlace`（被 GC） |
| `Arm_Aim()` | 零调用者；`Arm_Act_StartAim` 也不在 ELF |
| `Arm_Hold()` / `Arm_Release()` / `Arm_Laser()` | 零调用者 |
| `Arm_Home/Stow/MoveJoint/MovePose` | app 不直接调用，只经 `Arm_Debug_*`（`arm.c:328-346`） |
| `Arm_GetLastError()` | 仅调试视图内部调用（`arm.c:408`） |
| `Arm_IsBusy()` | app_debug 调用（`app_debug.c:394-430`） |

结论：正式业务接口 `PickAndPlace/Aim/Hold/Release/Laser` 完全未接入；`Home/Stow/Joint/Pose` 只存在于调试路径。

### 13.2 Arm_Update 推进

- 每 5 ms 调 `Arm_Update()`（`app_tasks.c:94-96`）；内部顺序：Step → Servo → Fb → Motion → Act（`arm.c:66-74`）。
- `Act_Update` 只在 `ACT_BUSY` 运行（`arm_action.c:379-384`）；`Motion_Update` 只在 `MOTION_BUSY` 运行（`arm_motion.c:250-253`）；`Fb_Update` 只在 `poll_active` 时轮询（`joint_fb.c:92-100`）。
- 动作一旦启动，任务层会推进；但 `Arm_Update()` 不会自动开始动作，必须有人调用 `Arm_Home/Stow/...`。
- 空闲时 `Arm_Fb_Update()` 不主动回读，关节角可能是旧的。

## 14. 资源：RAM/Flash

依据 `.pio/build/genericSTM32F103VE/firmware.elf`：

```text
text 58340
data   140
bss  16880
dec  75360
```

- Flash：text+data = 58,480 B；bin = 58,492 B；约占 11.4%。
- RAM：data+bss = 17,020 B；加链接器保留 `._user_heap_stack=1,540 B`，约 18,560 B / 65,536 = **28.3%**。
- `.bss` 含：FreeRTOS 堆 `ucHeap=8,192 B`、Idle 栈 512 B、Timer 栈 1,024 B、OLED 显存 1,024 B、底盘队列缓冲/控制块约 496 B、各驱动静态状态。
- HANDOVER 旧记录 15,472 B/23.6%、57,988 B/11.1%；当前构建加入 vision_link 后约 18.5 KB/28.3%。
- 两种口径差异来自是否计入 `._user_heap_stack`；以链接器自然段统计为准。

### 14.1 新增 mission 预计 RAM

- 并入 MainTask：静态事件/状态约 100~500 B，无独立栈，但叠加 MainTask 调用深度。
- 独立任务：1 KB 栈 + TCB 约 92 B ≈ 1.1 KB 堆；8~16 条事件队列 0.2~0.5 KB；合计约 1.3~1.6 KB。
- 当前 5 个任务约 2,560 B 栈 + 460 B TCB ≈ 3,020 B；8,192 B 堆剩余约 5.1 KB。新增 1 KB 栈 mission + 队列仍能放下，RAM 不是主要阻塞；Flash 余量约 450 KB。

## 15. 512 B 任务栈评估

- 5 个任务均 `128×4=512 B`（`main.c:57-89`）；`configMINIMAL_STACK_SIZE=128`、`configTOTAL_HEAP_SIZE=8192`（`FreeRTOSConfig.h:66-67`）；`configCHECK_FOR_STACK_OVERFLOW=2`（`:154`）；溢出钩子 `bkpt` 后关中断（`freertos.c:60-70`）。
- 栈帧估算（S18 启动 Pose 动作最深链）：
  - `App_MainTask` 8 B
  - `App_Debug_Update` 24 B
  - `Arm_Debug_MoveTestPose` 24 B
  - `Arm_Act_StartPose` 24 B
  - `act_begin` 24 B
  - `Arm_Motion_StartJoint` 104 B
  - `Arm_Servo_SetPwmSync` 112 B
  - `servo_send` 16 B
  - `HAL_UART_Transmit` 32 B
  - 合计约 **368 B**
- 加 Cortex-M3 异常入栈约 32 B、HAL 内部等待函数，实际约 **400~430 B**；512 B 只剩约 80~110 B，而 overflow check 还要保留栈底 16 B 模式。
- CarTask 约 200 B 较安全；EncTask 约 150 B，但 yaw 超过约 200 rad 时 libm 大参数规约 372 B 栈帧可能接近 512 B；OLEDTask 约 300 B。
- 结论：mission 并入 MainTask，512 B 不足；建议 MainTask 提升到 **1 KB** 并显示 `uxTaskGetStackHighWaterMark()`（全工程无调用）。独立 task 也建议 1 KB 栈、约 1.5 KB 堆。overflow check 只在上下文切换时发现栈底破坏，不能替代高水位测量。

## 16. 构建、中断、并发与阻塞

### 16.1 构建

- `env:genericSTM32F103VE`、`framework=stm32cube`（`platformio.ini:12-14`）。
- `build_src_filter` 编译 Core 与 src（`:19-21`）；include 路径（`:24-29`）。
- 最近构建产物 15:48，vision_link 已纳入；无 git，改动前备份。

### 16.2 中断与 ISR

- 四个 UART 优先级均 5（`stm32f1xx_hal_msp.c:336,366,396,428`）；`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`（`FreeRTOSConfig.h:125`）；临界区可屏蔽。
- 陀螺/步进/舵机/视觉 ISR 均只做整数解析，无浮点、无 RTOS API（`gyro.c:112-183`、`step_uart.c:70-161`、`servo_bus.c:105-211`、`vision_link.c:368-502`）。
- `uart_rx` 先重新挂接收再处理字节（`uart_rx.c:82-97`），减少下一字节丢失。

### 16.3 UART 槽位

- `UART_RX_SLOTS=4`（`uart_rx.c:10`）；注册顺序 huart2 → huart1 → huart4 → huart3（`app_init.c:14-23`）；刚好占满；无空槽时静默失败（`uart_rx.c:41-70`）。未来新增 UART 必须扩展槽位。

### 16.4 MainTask 中的阻塞调用

- `servo_send()`：两次 `HAL_Delay(1)` + `HAL_UART_Transmit(...,20 ms)`（`servo_bus.c:213-226`）；由动作启动调用（`:289-327`）。
- `step_send()`：阻塞 `HAL_UART_Transmit(...,20 ms)`（`step_uart.c:176-187`）。
- 一次 PTP 启动正常数毫秒，最坏错误超时可达数十毫秒；`Arm_Servo_StopAll()` 连发 3 条，最坏数十毫秒（`servo_bus.c:501-509`、`arm.c:76-82`、`arm_motion.c:66-72`）。
- 视觉 `vision_send_raw()` 20 ms 超时（`vision_link.c:506-540`）；每拍最多 4 帧，最坏 80 ms（`vision_link_config.h:32-36`）；正常 5 B 约 0.43 ms。
- `Chassis_Calibrate()` 初始化阶段阻塞最多 3 s（`app_tasks.c:67-68`、`chassis.c:46-62`）；陀螺 Init 在调度器前 `HAL_Delay(200)`（`gyro.c:214-220`）。

### 16.5 并发与共享状态

- 底盘指令/完成用 CMSIS 队列，生产者 MainTask、消费者 CarTask（`ctrl.c:151-159,173-200,565-616`），队列线程安全。
- **`s_planned_heading` 在 Submit（MainTask）累加、Ctrl_Update（CarTask）读取，无临界区**（`ctrl.c:185-186,382-386`）；32 位 float 读写原子，但"先累加后入队"顺序可能让控制层在旧段执行期间看到新段目标。
- `s_telem` 读写均有临界区（`ctrl.c:227-243,735-745`），正确。
- 路径事件单槽；`s_task_done_id` 可能跨任务写，单字节 volatile 基本安全，但无队列。
- 视觉事件/追踪槽/ACK/RX 队列均有临界区；ISR 只写追踪值槽、ACK 位图、RX 队列。
- 机械臂状态机全在 MainTask，无跨任务竞争；但阻塞 UART 会延长周期。

### 16.6 初始化顺序

- `App_Init()` 在 `osKernelInitialize()` 前（`main.c:158-162`）；`Chassis_Ctrl_InitRtos()` 在之后、建任务之前（`main.c:176-178`、`app_init.c:25-28`）。
- 运行时优先级：CarTask Realtime、EncTask AboveNormal、MainTask Normal（`main.c:196-200`）。
- MainTask 启动后 `osDelay(500)` + `Chassis_Calibrate(3000)`（`app_tasks.c:65-69`），期间 CarTask/EncTask 已运行，车体必须静止或架空。

## 17. 交叉集成事实与接口矩阵

### 17.1 任务层耦合

- MainTask 顺序：键盘 → 调试 → 视觉 → 路径 → 路径事件 → Arm_Update（`app_tasks.c:83-96`）。
- `App_Debug_HandlePathEvents()` 无条件消费路径事件并 500 ms 自动通知（`app_debug.c:161-197`、`app_debug.h:51`）；mission 接入后必须移除或隔离。
- `Vision_Link_Update()` 被调用（`app_tasks.c:89`），但 `TakeEvent/GetTrack` 与所有请求 API 无业务调用者，事件队列积压到溢出。
- 机械臂正式动作无调用者；`Arm_Update()` 空转。
- 没有底盘-机械臂互锁。

### 17.2 接口调用矩阵

底盘：

| 接口组 | app 调用 | 说明 |
|---|---|---|
| Init/InitRtos/Calibrate/Sense/Control | 是 | `app_init.c:18,27`、`app_tasks.c:36,55,68` |
| Path_Update/TakeEvent/NotifyTaskDone | 是 | `app_tasks.c:92`、`app_debug.c:165,195` |
| Path_Start/SelectById/GetState | 仅 S14 分支 | `app_debug.c:531-542,1346` |
| Path_Abort/SelectByName/IsRunning/IsWaitingTask | 否 | 零调用者 |
| Chassis_Control_Stop | 否 | 零调用者 |
| 调试直行/旋转/圆弧/速度/脉冲 | 是，但 S3~S14 不可达 | `app_debug.c:512-691`、`:227-246` |

机械臂：

| 接口组 | app 调用 | 说明 |
|---|---|---|
| Arm_Init/Arm_Update | 是 | `app_init.c:19`、`app_tasks.c:96` |
| Arm_Stop/Arm_IsBusy | 是（调试页） | `app_debug.c:394-430` |
| Home/Stow/MoveJoint/MovePose | 仅经 `Arm_Debug_*` | `arm.c:328-346`、`app_debug.c:268-271` |
| PickAndPlace/Aim/Hold/Release/Laser | 否 | 零调用者；相关代码被 GC |
| GetLastError | 否（仅调试视图） | `arm.c:408` |

### 17.3 参数可信度汇总

| 类别 | 参数 | 结论 |
|---|---|---|
| 高可信（实测） | 编码器方向/极性、左右速度环、死区、R200 圆弧系数、J2/J3/J4 零位、关节限位 | 有代码与调试记录，仍建议复测 |
| 中可信（实测但单工作点） | 轮距 190、圆弧补偿、航向 KI、速度前馈 | 仅当前地面/速度/半径有效 |
| 低可信（占位/不一致） | 路径距离、扫描长度、`MM_PER_PULSE` 与理论冲突、工作空间、收拢姿态、PTP 参数 | 必须实车标定或重新定义 |
| 未定义 | 任务 `tool_deg`、激光 GPIO、夹爪 180/270、B/C/D 固定姿态 | 阻塞业务落地 |

### 17.4 总体判断

- **底盘**可用于单段调试和局部路线验证；连续跑图前必须先修 `end_speed` 停车逻辑、路径 fault/任务等待、事件队列、A 点事件、队列满处理和中止入口。
- **机械臂**可用于调试页单步验证；正式 Pick&Place/Aim 前必须完成三姿态标定、J1 真实回读、工作空间检查、夹爪时序和激光可用性处理。
- **资源不是瓶颈，栈是瓶颈**；MainTask 512 B 必须提升并做高水位测量。
- **mission 是当前最大空缺**：视觉 API、机械臂正式 API、路径事件都没有业务消费者。

## 18. 潜在死循环、未初始化、浮点与越界

### 18.1 未初始化变量

- **`[新发现]` `Chassis_Cmd_t cmd` 在三个调试函数中未初始化，且缺 `end_speed_mmps`**：
  - 直行 `ctrl.c:787-810`、圆弧 `ctrl.c:817-838`、旋转 `ctrl.c:840-861`；
  - 只有路径层的 `cmd` 做了 `memset`（`path.c:174,226`）；
  - 控制层在直行/圆弧读取该字段（`ctrl.c:355-360,478-480`）。
  - 影响：调试运动可能沿用栈上随机 end_speed，行为不确定。
- **`[新发现]` `Chassis_Debug_RunPulses()` 结束后 `s_pulse_hold` 永久保持 1**，`Chassis_Debug_Update()` 每 5 ms 重复制动，`s_debug_override` 一直为 1；只有按 7 调 `StopPulses()` 才释放（`chassis.c:152-196`）。忘记释放后，正式运动会被控制环的调试接管分支直接跳过（`ctrl.c:543-547`）。
- **`[新发现]` `s_servo_target[3]` 只在调试点动中更新**（`arm.c:24,59-61,192-224`）；正式 PTP 不更新它，S16 点动会从启动零位而非当前位置开始。
- **`[新发现]` `Math_Wrap180(NaN)` 返回 NaN**（两次比较都为 false）；`Arm_Limit_CheckJoints()` 对 NaN 全部通过（`arm_limit.c:20-35`），随后 `lroundf(NaN)`、PWM 计算属于未定义行为。正常 mission 不应产生 NaN，但入口无校验。
- **`[新发现]` `App_Dbg_FmtFloat()` 乘 scale 后强转 int32，无范围/NaN 检查**（`app_debug.c:741-763`）。当前显示范围有限，传感器异常值可能产生未定义结果。

### 18.2 死循环与无限等待

- `Math_Wrap180()` 的 `while` 对 `+Inf` 会无限循环，对 NaN 返回 NaN（`math_util.c:11-19`）。
- `Chassis_Calibrate()` 有 3 s 超时（`chassis.c:46-62`），不会永久阻塞。
- `Error_Handler`、`configASSERT`、栈溢出/malloc 失败钩子是安全停机死循环（`main.c:782-790`、`FreeRTOSConfig.h:138`、`freertos.c:60-81`）。
- `vision_ack_flush()` 位扫描必然找到置位位，最大 31，不越界（`vision_link.c:957-970`）。
- `vision_track_service()` 丢失计数受 `<5` 限制（`vision_link.c:925-930`）。
- **`Chassis_Path_Update()` 的 `SEG_TASK_WAIT` 是逻辑无限等待**（`path.c:496-504`）：不是 CPU 死循环，但任务链会永久停住且无超时/错误事件。
- `App_Debug_HandlePathEvents()` 的单槽 while 最多执行一次（`app_debug.c:165`）。

### 18.3 浮点与中断

- 陀螺仪 ISR 只做整数解析与 volatile 更新，浮点在 `GetFrame/GetYawDeg/GetWzDps` 任务上下文（`gyro.c:112-183,238-320`）。
- 步进/舵机 UART 字节回调均为整数/字符处理（`step_uart.c:70-161`、`servo_bus.c:105-211`）。
- 视觉字节 ISR 只做整数解析、XOR、队列写入，无 RTOS API、无浮点（`vision_link.c:368-502`）。
- UART 中断优先级均为 5（`stm32f1xx_hal_msp.c:336,366,396,428`），等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`（`FreeRTOSConfig.h:125`），`taskENTER_CRITICAL` 能屏蔽。
- 未发现 ISR 内浮点或阻塞调用。
- **`[推断]` 无界 yaw 送入 `sinf/cosf`**（`odom.c:159-161`）：`|yaw|` 超过约 200 rad（约 32 圈）时 libm 走大参数规约，`__kernel_rem_pio2f` 栈帧约 372 B；EncTask 512 B 栈存在长跑溢出风险。常规比赛可能达不到，长跑/连续旋转需警惕。

### 18.4 除零

主要除法均有保护：

- `Pid_Update()` dt 下限 0.0001（`pid.c:73-76`）。
- `Chassis_Odom_Update()` dt 下限 1 ms（`odom.c:66-73`）。
- `Chassis_Kin_Arc()` `|radius|<1` 走原地旋转（`kin.c:60-65`）。
- `ctrl_run_arc()` 先判 `|radius|>1`（`ctrl.c:464-496`）。
- `seg_estimate_ms()` 速度保证 >0（`path.c:51-58,60-104`）。
- `Arm_Kin_Inverse()` `rm<KIN_EPS` 与 ratio 越界返回 -1（`arm_kin.c:112-123`）。
- `Arm_Motion_StartJoint()` t_servo 最小 0.2 s，RPM 限 2~120（`arm_motion.c:143-181`）。
- `App_Dbg_FmtFloat()` scale 由 decimals 0~4 生成，不为 0（`app_debug.c:750-793`）。
- `Chassis_Kin_RadiusFrom()` 对近 0 角度有保护（`kin.c:72-79`）。

结论：未见确定除零路径；真正需要注意的是 `ARM_ARC` 反向速度与 NaN。

### 18.5 数组越界

- 路径表索引检查 `id < g_chassis_path_count`（`path.c:329-342`）；`s_index` 先判断再使用（`path.c:121-134`）。
- 步进帧缓冲 16 B，协议帧最大 8 B（`step_uart.c:30,54-68,100-113`）。
- 舵机行缓冲 24 B，长度 23 停止追加（`servo_bus.c:19,182-211`）。
- 舵机扫描 ID 数组 16，有边界检查（`servo_bus.c:56,747-765,782-790`）。
- 视觉 RX 队列 8 且为 2 的幂；载荷 4 B；ACK 位图 8 word，`cmd>>5` 最大 7；事件队列 8（`vision_link.c:20,35,89,109-111,315-318,322-364,564-586`）。
- `Arm_Debug_GetView()` 的 `servo_pwm[i-1]` 仅在 J2~J4 循环（`arm.c:392-406`）。
- `dbg_stpcpy()` 无长度参数（`app_debug.c:694-702`）：当前最长约 22 字符未越界，但未来新增字符串有风险。
- `servo_parse_line()` 对 `s_line[1..3]` 非数字不校验，会算出错误 ID 但不越界（`servo_bus.c:119-121`）。

结论：未发现确定性数组越界；风险集中在无长度参数的 `dbg_stpcpy()`、NaN 后的未定义行为、未做有限性检查的数学函数。

## 19. 落地阻塞清单

### A. 必须实车标定

1. **底盘距离系数一致性**：`MM_PER_PULSE` 0.002242 与理论 0.003324、S5 注释"约 0.003"、`DEBUG_NOTES.md:16` 的 0.002298 不一致（`chassis_config.h:44-47`）。需确认轮径/减速比/编码器线数或用 3000~5000 mm 重新标定。
2. **航向串级参数**：外环 KI=2.5、内环 1.5 仍整定（`chassis_config.h:104,109`、`DEBUG_NOTES.md:193-199`）；需按 1400 mm、200/250/300 mm/s 记录段末 yaw 与横向偏差。
3. **圆弧补偿**：`V_SCALE_L/R`、`W_SCALE`、`Kr` 只在 R200/200 mm/s 标定（`chassis_config.h:148-155`）；左右转系数 1.23 vs 1.49。
4. **轮距/侧滑修正**：190 mm 为卷尺实测，无侧滑补偿（`chassis_config.h:15`、`odom.c:82,155-156`）；建议标定有效轮距或把融合权重降到 0 对比。
5. **陀螺寄存器与零偏**：寄存器地址仍 TODO（`chassis_config.h:191`）；确认 0x69/0x03/100 Hz 配置生效；`Chassis_Calibrate()` 返回值被忽略（`app_tasks.c:68`），失败不阻止运行。
6. **路径表全部距离/扫描长度**：占位（`path_table.c:9`）；表 0/A→B/C→D 注释与数值不一致。
7. **机械臂几何/零位/方向/限位**：按三姿态验证（`ARM_DEBUG_NOTES.md:89-99`）：零位 TCP r≈360、z≈130 或 90；q3=+90 时 r≈65、z≈315 或 275；q4=−90 时 r≈335、z≈−5 或 −45；确认 `ARM_ELBOW_OFFSET_SIGN`、`ARM_IK_ELBOW`、q3=0 定义。
8. **夹爪实际行程**：833/1278/1500 µs 基于 270° 假设（`arm_config.h:58-66`）；若 180° 需重算。
9. **PTP 参数**：90/120 °/s、0.2 s、2°、1500 ms、3 次稳定均为设计值（`arm_config.h:122-130`）；需测到位误差、超调、稳定时间；`ARM_PTP_STEP_W_MAX_DEGPS` 未使用；跨总线 2~5 ms 偏差需确认可接受。
10. **J1 步进细分/速度/加速度/极性**：3200、30 RPM、10、半双工极性均为 TODO（`arm_config.h:16,18-19,54-56`）。

### B. 必须写代码

底盘：

1. **真正实现或移除段间平滑**：`end_speed_mmps` 非零仍会在段末 `ctrl_arrival()` + 反向脉冲 + `ctrl_finish()` 停车（`ctrl.c:275-287,355-360,392-395,655-667,271-272`）；`DEBUG_NOTES.md:187` 的"段间平滑"不成立。
2. **修复调试命令未初始化**：三个 `Chassis_Cmd_t` 加 `memset` 并补 `end_speed_mmps`（`ctrl.c:787-810,817-838,840-861`）。
3. **fault 传播到路径层**：`path.c:454-468` 增加 `st.fault` 检查，走 `path_error()`；`Chassis_Ctrl_WaitDone()` 无调用者可删除或使用。
4. **任务等待超时与错误恢复**：`SEG_TASK_WAIT` 增加超时（`path.c:496-504`）；明确 `NotifyTaskDone` 的错误点号/重复通知策略；`NotifyTaskDone(0)` 无法完成 0 号点（`path.c:498`）。
5. **路径事件改队列**：单槽会覆盖（`path.c:48-49,106-111`）；至少 4~8 条环。
6. **队列满回滚与命令确认**：`Chassis_Ctrl_Submit()` 先改计划航向、后入队且忽略返回值（`ctrl.c:185-199`）；需入队成功后再累加或失败回滚、返回 int、路径层立即报错。
7. **修复 stale finished/seq 竞态**：新命令提交时清 `s_telem.finished`，或增加命令接收状态/序号确认（`ctrl.c:245-260,863-877`、`path.c:376-384,454-468`）；`Chassis_Ctrl_Stop()` 清 telem（`ctrl.c:863-877`）；路径序号不要每次 Start 从 1 重新开始。
8. **补 A 点事件**：表 0 增加 `PATH_TASK`（`point_id='A'`）或最后一段设点号（`path_table.c:17-26`）。
9. **恢复路径入口与中止**：S3~S14 全不可达（`app_debug.c:23,227-246`）；S14 注释"9=中止"与实际不符（`:512-546`）；增加真正的路径中止键调用 `Chassis_Path_Abort()`（当前 0 调用者，`path.c:389-395`）；增加 `Chassis_Control_Stop()` 入口（0 调用者，`chassis.c:74-77`）。
10. **移除 500 ms 调试自动放行**（`app_debug.c:191-196`、`app_debug.h:51`）。
11. **陀螺标定失败处理**：检查 `Chassis_Calibrate()` 返回值，失败禁止自动跑图（`app_tasks.c:68`）；增加陀螺在线/速率确认。
12. **圆弧反向速度修复**：`ctrl.c:485-488` 的 `v_target` 最小钳位会破坏负速度，需按符号分别处理。
13. **调试状态陷阱**：`Chassis_Debug_Update()` 的 pulse hold 不应永久保持 debug override（`chassis.c:174-196`）；`Chassis_Ctrl_DebugRunSpeed()`/`SetDebugOverride()` 应清空命令队列（`ctrl.c:759-773,747-757`）。

机械臂：

14. **J1 真实反馈**：运动完成后读 `Arm_Step_GetPositionDeg()` 与软件累计比较，超差报 COMM（当前函数 0 调用者，`step_uart.c:352-369`）；检测步进复位/离线后重新在线，不要沿用 `s_j1_pos_deg`（`joint_fb.c:22-24,156-160`）；运动启动时不要把 J1 软件位置直接当到位（`arm_motion.c:222-225`）。
15. **回读新鲜度与稳定采样**：记录每轴回读时间戳，只有"新样本"才计入 `s_settle_count`（`arm_motion.c:297-307`、`joint_fb.c:137-178`）；当前 3 次稳定是 3 个 5 ms 控制拍。
16. **工作空间检查**：`Arm_Act_StartPose()`/`StartAim()` 增加 `Arm_Limit_CheckPose()`（`arm_action.c:268-278,340-350`）；当前只有 Pick&Place 使用 workspace（`:304-307`）。
17. **Pick&Place 时序与超时**：夹爪 OPEN/GRASP/OPEN 后增加 `ARM_GRIPPER_MOVE_MS` 等待（`arm_action.c:114-115,149-152,199-202`、`arm_config.h:67`）；每步超时；`approach_mm` 范围检查。
18. **Aim 激光语义**：完成时检查 `Arm_Laser_IsAvailable()`，不可用返回错误而非 DONE（`arm_action.c:398-403`、`laser.c:45-48`）；明确自动关激光策略；启动新动作时是否强制关激光。
19. **动作启动 BUSY 契约**：`act_begin()` 返回 BUSY 时无动作状态和超时（`arm_action.c:53-56`）；要么内部缓存重试，要么在 `arm.h` 明确要求调用方重试并让 `Arm_IsBusy()` 反映"待启动"（`arm.c:84-87`）。
20. **故障锁存**：`Arm_Stop()` 清错误（`arm_action.c:437-446`）；增加 ESTOP/FAULT 锁存，显式复位才清除；`ARM_ERR_ESTOP` 已定义未使用（`arm_types.h:22`）。
21. **动作超时/错误上报细化**：`Arm_GetLastError()` 只返回最后一个错误且成功启动会清空；增加错误历史/故障码；`act_fault()` 应记录触发动作/步骤（`arm_action.c:37-45`）。
22. **调试页入口**：增加 S15~S17 运行时切换或恢复 S3~S14；增加 GRASP 测试键（`app_debug.c:375-376`）；修复 `s_servo_target` 与正式运动不同步（`arm.c:24,59-61,192-224`）。
23. **任务层与视觉事件消费**：新增 mission，消费 `Vision_Link_TakeEvent()`、`GetTrack()`、`Chassis_Path_TakeEvent()`。
24. **栈与可观测性**：MainTask 栈提升到 1 KB，并显示 `uxTaskGetStackHighWaterMark()`；当前只有溢出钩子（`FreeRTOSConfig.h:154`、`freertos.c:60-70`）。

### C. 必须定业务规则

1. **A/B/C/D 任务链时序**：`SCAN_READY` 后 300 ms 就倒回（`path.c:294-302`）。取图是在 `SCAN_READY` 还是倒回后的 `ARRIVE_POINT`？若在 `SCAN_READY`，300 ms 是否够（协议单次请求最长 1 s）？若在 `ARRIVE_POINT`，路径会无限等待，任务层必须保证最终通知。
2. **`NotifyTaskDone` 调用条件**：视觉成功、机械臂成功还是超时降级？失败时重试、跳过还是中止？路径层无 fault 分支、无任务失败事件。
3. **A 点是扫码还是仅途经**：当前无点事件；需决定是否停车扫码、扫码超时/重试。
4. **B/C/D 的 `tool_deg` 与固定姿态**：`Arm_Pose_t.tool_deg` 已存在（`arm_types.h:39`），任务固定值未提供（`HANDOVER.md:249`）。
5. **激光开关策略**：Aim 成功后何时关？mission、定时器还是动作层自动关？`C_TARGET_DOWN` 无人消费。
6. **急停/中止策略**：按键急停是否同时停底盘、机械臂、激光、夹爪？急停后是否允许自动恢复？
7. **夹爪 270°/180° 实际定义**：若 180°，OPEN/GRASP/CLOSE 物理角度需重定，不能只改 macro。
8. **机械臂与底盘联动约束**：行车前是否必须 Stow、机械臂动作时底盘是否必须停？当前无 interlock，二者在同一 MainTask 顺序执行（`app_tasks.c:92-96`）。

### D. 仅需现场数据

1. 地图距离、扫描段长度、A/B/C/D 坐标、转弯半径。
2. 视觉从触发到返回的延迟、帧率、丢帧率。
3. 300 ms 扫描停留是否足够（相机曝光/识别时间）。
4. 地面摩擦/打滑率、左右履带差异、有效轮距。
5. 夹爪实际 180°/270° 与开/夹/合真实 PWM。
6. 激光 GPIO 引脚与极性。
7. 机械臂各轴齿隙、重力下垂、重复定位误差。
8. 5 分钟赛程下各段耗时与总预算。
9. 整车 250×250×300 mm 尺寸验证与收拢姿态碰撞检查。

## 20. 已知限制对照（确认仍存在）

| 问题 | 文档位置 |
|---|---|
| 航向 KI 仍待整定 | `DEBUG_NOTES.md:193-199`、`HANDOVER.md:128` |
| 300 mm/s 高速打滑偏航 | `DEBUG_NOTES.md:132-137` |
| 圆弧补偿仅 R200/200 mm/s 有效 | `chassis_config.h:148-155`、`DEBUG_NOTES.md:169-175` |
| 里程计打滑时偏大，不能用 DST 判到位 | `DEBUG_NOTES.md:117-123` |
| 轮距 190 无侧滑修正 | `CHASSIS_COMPARISON.md:210-217` |
| 固定 dt 与实测 dt 混用 | `CHASSIS_COMPARISON.md:124,243-244` |
| `Submit()` 忽略队列满 | `CHASSIS_COMPARISON.md:125` |
| 航向积分跨段保持 | `CHASSIS_COMPARISON.md:219-225` |
| `CHASSIS_PATH_PERIOD_MS` 未使用 | `CHASSIS_COMPARISON.md:127` |
| 路径距离/扫描长度占位 | `path_table.c:9`、`HANDOVER.md:131` |
| A 点无 `PATH_EV_ARRIVE_POINT` | `HANDOVER.md:132,204` |
| 路径层不检查 fault | `HANDOVER.md:134` |
| S14 路径页无运行时入口 | `HANDOVER.md:134`、`app_debug.c:23` |
| 事件单槽风险 | `HANDOVER.md:222` |
| mission 未实现 | `HANDOVER.md:112` |
| 视觉链路未实机联调 | `HANDOVER.md:182` |
| 机械臂正式动作无业务调用方 | `HANDOVER.md:152` |
| IK 20 mm 偏置/TCP 未验证 | `HANDOVER.md:153`、`ARM_DEBUG_NOTES.md:5,89-99` |
| 关节角/限位/收拢姿态为实测近似 | `HANDOVER.md:154`、`ARM_DEBUG_NOTES.md:5` |
| 夹爪测试键未加 | `HANDOVER.md:156` |
| 激光未分配，`ARM_LASER_ENABLE=0` | `HANDOVER.md:157`、`arm_config.h:69-71` |
| J1 掉电丢位置、上电手动对正 | `HANDOVER.md:157`、`arm_config.h:21-24` |
| 机械臂动作未与 mission/vision 联调 | `HANDOVER.md:158` |
| 统一安全 FSM 未实现 | `ARM_PLAN_REVIEW.md:124-125` |
| 回读慢积分校正/齿隙/重力补偿未实现 | `ARM_PLAN_REVIEW.md:41-43` |
| PTP 跨总线 2~5 ms 偏差 | `ARM_DEBUG_NOTES.md:212` |

本次新增、文档中未明确记录的重点问题：

- `end_speed` 非零仍会停车，段间平滑未真正实现（`ctrl.c:355-360,392-395,655-667,271-272`）。
- 调试直行/圆弧/旋转的 `cmd.end_speed_mmps` 未初始化（`ctrl.c:787-810,817-838,840-861`）。
- `MM_PER_PULSE` 与理论值/S5 注释/DEBUG_NOTES 三套口径不一致（`chassis_config.h:14-47`、`app_debug.c:651-665`、`DEBUG_NOTES.md:16,185`）。
- 完成状态的 stale `finished` + seq 复用竞态（`ctrl.c:245-260,863-877`、`path.c:376-384,454-468`）。
- 路径中止键注释与实现不一致，`Chassis_Path_Abort()` 无调用者（`app_debug.c:515,526-533`、`path.c:389-395`）。
- S3~S14 全部底盘调试页在运行时不可达（`app_debug.c:23,227-246`）。
- J1 位置在命令下发时即被软件置为目标，无法发现丢步/复位（`arm_motion.c:222-225`、`joint_fb.c:156-160`）。
- `ARM_PTP_SETTLE_SAMPLES=3` 实际是 3 个控制拍，不是 3 次新回读（`arm_motion.c:297-307`）。
- Pose/Aim 不检查工作空间，只检查关节限位（`arm_action.c:268-278,340-350`）；workspace 只在 Pick&Place 使用（`:304-307`）。
- Pick&Place 夹爪动作后无等待，`ARM_GRIPPER_MOVE_MS` 未使用（`arm_action.c:114-115,149-152,199-202`、`arm_config.h:67`）。
- Aim 在激光不可用时仍报告成功，成功后无自动关激光（`arm_actuator.c:48-53`、`arm_action.c:398-403`、`laser.c:45-48`）。
- 调试舵机目标缓存 `s_servo_target` 与正式 PTP 不同步（`arm.c:24,59-61,192-224`）。
- `Arm_Stop()` 清空错误，急停与故障不可区分（`arm_action.c:437-446`、`arm.c:89-104`）。
- `[推断]` 无界 yaw 送入 `sinf/cosf`，长跑可能触发 libm 372 B 栈帧（`odom.c:159-161`）。

## 21. 第二份审查的结论

- **底盘**：单段控制与驱动成熟，可作调试基线；但连续路径的"平滑衔接"未真正实现，fault/任务等待/队列/事件/中止均有缺口，A 点事件缺失，S3~S14 无运行时入口。当前适合"单段实车调试"，不适合"无人自主跑完整链"。
- **机械臂**：驱动层与单动作状态机完整，但正式动作无业务调用方；Pick&Place/Aim 甚至不在当前链接产物中。J1 是软件假回读，工作空间检查不完整，夹爪/激光时序与语义未闭环。当前适合"调试页单步验证"，不适合"任务级自动抓取/打靶"。
- **资源**：审查代理读到 RAM 28.3%、Flash 11.4%（需与最终构建复核）；新增 mission 的 RAM/Flash 不是阻塞；512 B 栈对 MainTask 已接近上限，接入 mission 事件处理与浮点后风险很高，建议提升到 1 KB 并做高水位测量。
- **建议顺序**：
  1. 修 `end_speed` 停车逻辑与调试命令未初始化；
  2. 修路径 fault/等待超时/事件队列/中止入口；
  3. 补 A 点 PATH_TASK 与路径入口，恢复 S14；
  4. 实车标定机械臂三姿态，确认偏置符号/TCP/肘部；
  5. 修 J1 真实回读、夹爪时序、激光可用性与自动关闭；
  6. 实现 mission，移除 500 ms 假完成，接入视觉事件；
  7. 提高 MainTask 栈并显示高水位。

---

# 三、视觉协议与工程一致性（已完成）

> 方法：协议 v1.5.1 与 `vision_link` 逐行对照、`-Wall -Wextra` 语法检查、ELF 符号检查。
> 未做实机与视觉端联调；"能发出/能解析"是源码层结论，不是现场时序结论。
> 构建状态备注：审查代理未运行完整 `pio run`，且其看到的 ELF 时间戳早于源码 7 秒。
> 公共 API 在 ELF 中被链接器 GC，是因为当前无业务调用者，不代表源码未编译；
> 但当前 ELF 的 RAM/Flash 数字与最后一次构建记录不一致，需重新完整构建后复核。

## 1. 协议一致性

### 1.1 帧格式、XOR、int16 大端

**结论：符合。**

- 协议：`通信协议_视觉与MCU_v1.5.1.md:8-12`。
- 发送：`vision_link.c:518-529`，`A5 CMD LEN DATA XOR`，XOR 覆盖 CMD/LEN/DATA。
- 解析：`vision_link.c:437-499`，CMD 起算、LEN 与 DATA 逐字节异或。
- 大端：`vision_link.c:117-128`。
- 视觉→MCU 最大 LEN=4（`vision_link_config.h:16`）；0x14 的 6B 只在发送侧（`:18`）。
- 协议未规定畸形帧是否回 ACK；实现选择不回、只计数。

### 1.2 MCU→视觉全部命令

| CMD | 发送位置 | 载荷 | 结论 |
|---|---|---|---|
| 0x10 | `vision_link.c:1127` | 0 | 正确 |
| 0x28/0x38/0x48 | `vision_link.c:1146,1161` | 0 | 正确 |
| 0x29/0x39/0x49 | `vision_link.c:1214,1232` | 0 | 正确 |
| 0x20/0x30/0x40 | `vision_link.c:1166,1184` | 0 | 正确 |
| 0x22/0x32/0x42 | `vision_link.c:1289-1293` | 2B Hz 大端 | 正确 |
| 0x24/0x34/0x44 | `vision_link.c:1320,1334` | 0 | 正确 |
| 0x25 / 0x27 | 同上（target=B_BUCKET） | 2B / 0 | 正确 |
| 0x14 | `vision_link.c:1005` | 6B xyz 大端 | 载荷正确，方向语义有歧义 |

问题：

- **P2（高）MCU→视觉命令没有送达确认**。协议只允许 MCU 回 ACK，视觉不回；0x28/0x22/0x24/0x25/0x27/0x34 发一次就不管。丢一帧可能导致检测器未选、视觉继续推流到 60s、或 0x34 后开激光不触发。需协议层拍板是否增加正向确认或 MCU 侧重发。
- **P12（高）同目标 TrackStart 不重发 0x22**。`vision_link.c:1284-1287` 在目标已激活时直接返回 OK；5X 时 `vision_link.c:939-944` 只发事件、不清 active。mission 收到 5X 后同目标重启会得到 OK，但视觉收不到新 0x22。
- **P13（中）B 物体切 B 桶必须先 Stop**。`vision_link.c:1280-1283` 在 active 为 B_OBJECT 时 TrackStart(B_BUCKET) 返回 BUSY；协议 B 放置段 0x25→0x26→0x27 未明确要求先 0x24。
- **P11（高）0x34→开激光、0x35/3s→关激光未落地**。`vision_link.c:1315-1347` 只发 0x34；`:796-799` 只发 C_TARGET_DOWN 事件；全工程无 3s 激光超时；`ARM_LASER_ENABLE=0`、`laser.c:26-33` 为空操作。
- **P21（高）没有公开的 `R_tool→base` / 当前 TCP 位姿 API**。协议要求用当前位姿 + 旋转矩阵换算 dx/dy（协议 §2）；`arm.h:26-44` 只有动作接口，`Arm_Kin_Forward` 在内部层 `arm_kin.h:17`，`Arm_Fb_GetJoints` 在 `joint_fb.h:18`。mission 无法做坐标变换，也无法填 0x14。
- **P25（低）hz 上限 200 为自设**，协议只建议 50。
- **E：全部命令 API 当前 0 调用者**，ELF 中符号已被 GC。

### 1.3 视觉→MCU 全部命令

| CMD | 处理位置 | 校验 | 事件 | 结论 |
|---|---|---|---|---|
| 0xFE | `vision_link.c:393-400` | LEN=0 | 刷新链路 | LEN≠0 仍刷新（P9） |
| 0x11 | `vision_link.c:714-725` | LEN=0 | SCAN_OK（仅 pending scan） | 正确 |
| 0x12 | `vision_link.c:727-746` | LEN=1，code 1..6 | SCAN_ERROR / VISION_ERROR | 分类有问题（P3） |
| 0x13 | `vision_link.c:748-762` | LEN=0 | TARGET_LOST（仅追踪） | 正确；无 target 字段（P10） |
| 0x21/31/41 | `vision_link.c:764-778` | LEN=1，0..3 | MIDLINE（zone 匹配） | 正确 |
| 0x2A/3A/4A | `vision_link.c:780-794` | LEN=1，0..2 | ORDER | 未检出=1 有歧义（P4） |
| 0x23/26/33/43 | `vision_link.c:377-391` | LEN=4 | 更新最新值槽 | 正确 |
| 0x35 | `vision_link.c:796-799` | LEN=0 | C_TARGET_DOWN | 正确 |

问题：

- **P3（高）0x12 没有事务关联，分类靠猜**。`vision_link.c:727-746` 只看 pending 是否为 SCAN；否则发 VISION_ERROR 且不清 midline/order pending。中位线 pending 期间收到 0x12 会每 500 ms 永久重发；顺序 pending 只能等 3 s 兜底；扫码 pending 期间 code5 会被误当 SCAN_ERROR 并清追踪。
- **P4（高）0x2A"未检出回 1"与"名次 1"不可区分**。协议 §5.1 写未检出回 1，§6 又写 3 s 兜底为 1；实现正常响应 `flags=0`，仅超时兜底 `flags=0x01`（`vision_link.c:780-789`、`:876-877`）。mission 可能选错预设目标。
- **P9（中）心跳 LEN≠0 仍刷新链路**。`vision_link.c:393-400` 计 err_payload 后 return，但链路时间已在 `:372-374` 更新。
- **P8（中）任意 XOR 正确帧都刷新链路**，包括未知 CMD、非法载荷、错误方向帧；`rx_frames` 也不是合法帧数。
- **P10（低）0x13/TRACK_LOST 事件没有 target 字段**；B 物体与 B 桶同属 zone B，mission 只能靠自身状态推断。
- **P22（低）非活动目标的偏移仍写槽位**；`GetTrack` 要求 active 匹配，不会被误用。

### 1.4 ACK 豁免、ACK 帧、畸形帧

- 豁免表 `vision_link.c:298-313` 与协议 §1 一致（0x23/26/33/43、0xFE、0xFF、0x14）。
- ACK 帧 `vision_link.c:518-529` 为 `A5 FF 01 CMD XOR`，与协议示例一致。
- 只有成功入队才置 ACK 位（`vision_link.c:421-430`）；XOR 错、LEN 越界、未知 CMD、非法载荷不回 ACK。

问题：

- **P18/C7（中）同一 CMD 合并 ACK**。256 bit 位图同 CMD 只置一位，每拍最多 2 个；若视觉要求每帧一个 ACK，会少 ACK。
- **P17（低）畸形帧不回 ACK，协议未定义**。
- **P23（低）0x11 会回 ACK**，需确认视觉端是否处理。
- **P24（低）0x14/0xFF 在豁免表中逻辑冗余**；协议表把 0x14 列为 MCU 回 ACK 的豁免本身自相矛盾。

### 1.5 心跳、失联、恢复

- 500 ms 超时 `vision_link_config.h:28`、`vision_link.c:590-606`。
- 在线状态机 `vision_link.c:681-703`：首次上线不报 RECOVERED；掉线报 LINK_LOST；恢复报 RECOVERED。
- 失联清理 `vision_link.c:650-679`：pending 发 TXN_ABORTED 并清；追踪清 active 与丢失计数；清 pose、ACK 位图、RX 队列、解析器。
- 恢复后不自动重发，需 mission 重新发起。

问题：

- **P19（高，系统级）"回展开态"没有实现**。协议要求失联停追踪、回展开态；通信层只上报事件，没有调用机械臂/底盘。mission 消费 LINK_LOST 后需执行 `Arm_Stow`/停车，并定义恢复流程。
- **P8 同上**：任意 XOR 正确帧刷新链路，可能掩盖真实失联。
- **C 级边界：长阻塞会掩盖"已恢复的失联"**。`vision_link_state_service` 只在 Update 时采样；MainTask 被阻塞超过 500 ms 且期间视觉掉线又恢复，下次 Update 看到"最近有帧"，不会补发 LINK_LOST/RECOVERED。
- **P20（低）没有本地 60 s 追踪看门狗**；只处理视觉发来的 0x12(code5)。

### 1.6 超时与重发语义

- **扫码**：首帧立即发；每 1000 ms 重发；第 3 次后 1000 ms 超时（`vision_link.c:824-851`、`:1112-1142`）。时序 0/1/2 s 发送，3 s 超时。
- **中位线**：首帧立即发；每 500 ms 无限重发，直到响应或取消（`vision_link.c:853-871`、`:1164-1199`）。
- **顺序**：首帧立即发；3 s 无回包发 ORDER value=1、flags bit0=1（`vision_link.c:873-880`、`:1212-1247`）。
- **追踪**：新样本清丢失窗口；500 ms 无样本发 TARGET_LOST；连续 5 个窗口发 TRACK_LOST_5X（`vision_link.c:888-945`）。
- **0x13 与窗口不重复计数**：共用 `s_loss_reported`，谁先到谁置位，新样本重置。

问题：

- **P6（中）扫码"1 s×3 重发"歧义**：当前总共 3 次，也可能是首帧 + 3 次重发 = 4 次。
- **P5（高）中位线收到 0/2/3 也清 pending**（`vision_link.c:768-773`）；协议说"反复请求直到成功"。mission 只调一次会永远等不到 1。需拍板由 mission 重调还是通信层保持 pending。
- **P7（中）"连续 5 次"定义歧义**：当前 5 个 500 ms 无样本窗口（约 2.5 s），不是 5 次 0x13。
- **C11（中）响应与超时同拍竞态**：`vision_rx_drain()` 在 `vision_pending_service()` 之前（`vision_link.c:1098-1104`）；drain 与 deadline 检查之间到达的帧可能被超时/兜底抢先，下一拍才当 late_responses。

### 1.7 追踪流、DX/DY 符号、z

- 4 个槽 `vision_link_config.h:24`、`vision_link.c:44-50,82`。
- ISR 只更新最新值，不入队、不回 ACK（`vision_link.c:377-391`）。
- `GetTrack` 临界区读，年龄 ≥500 ms 返回 0（`vision_link.c:1364-1411`）。
- 只有 dx/dy，无 z，符合协议。

问题：

- **P21 无消费者、无坐标变换**。
- **符号约定未验证**：代码不翻转符号，完全信任视觉端；现场必须验证 x 左 y 上，否则机械臂反向微调。
- **z 缺失**：mission 必须用预设 z/接近策略，不要误用 0x14 的 z。
- 50 Hz 槽位单生产者/单消费者安全，没有半写问题。

### 1.8 0x14 限速与方向

- 20 ms 限速（`vision_link_config.h:61`）、每拍最多一帧、覆盖最新值、失败丢弃（`vision_link.c:986-1017`、`:1349-1362`）。
- **P1（高，需拍板）方向语义**：协议表写 MCU→上位机，§5.3 又写视觉只记录；实现发到 USART3 视觉链路并在 ACK 豁免表列 0x14。需确认上位机是否同一 USART3。
- **P14（低）离线时丢 pose**：`ReportTcpPose` 先查 IsOnline，离线直接返回，不缓存。
- **P26（低）50 Hz 自设**，协议未规定频率；带宽占用约 4.3%。
- **无调用者**：0x14 当前永远不会发出。

## 2. vision_link 新代码审查

### 2.1 帧解析器

正确：状态机 `vision_link.c:22-26,433-502`；数据区 0xA5 按数据处理；LEN 越界立即回 HEAD；残帧 100 ms 超时；XOR 错回 HEAD；失步恢复。

问题：

- **C1（低）截断帧会吞下一帧的 0xA5**。残帧停在 VF_DATA 时，下一帧 A5 被当数据，直到 XOR 错或 100 ms 超时；50 Hz 下可能丢 1~2 帧。协议只有 XOR，这是固有限制。
- **C2（低）残帧超时有二次临界区竞态**（`vision_link.c:633-644`）：读与复位分两次临界区，中间可能被 ISR 推进并截断新帧。建议判断+复位放同一临界区。
- **C3（低）噪声计数低估**：只在 HEAD 且 synced 时计一次，CMD/LEN/DATA 中的噪声字节不计。
- **P8/C4（中）链路时间在命令/载荷校验前更新**。

### 2.2 SPSC 环形队列

正确：写槽位 → `__DMB()` → 发布 head；先看 tail → `__DMB()` → 读槽位 → 推进 tail；满/空判断正确；8 槽 2 的幂；失联清队列在临界区内。单生产者/单消费者、Cortex-M3 单核、DMB + volatile 足够。

问题：

- **C5（极低）pop 无条件拷贝 4 字节**，len=0/1 时多读旧数据，不越界、无实际影响。
- **C6（低）没有静态断言保证队列长度是 2 的幂**；改成 10 会静默出错。
- 满时丢帧且不 ACK，设计合理；但 0x35 等安全帧丢失后果较大。

### 2.3 ACK 位图

正确：ISR 置位，任务临界区读/清；USART3 优先级 5 与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` 使 BASEPRI 能屏蔽 ISR。

问题：

- **C7 同 CMD 合并**：见 P18。
- **C8（中）先清位再发送，发送失败/无预算则 ACK 永久丢失**（`vision_link.c:967-968,978`、`:547-550`）。若 pending_service 已耗尽预算，ack_flush 仍先清位再返回 NO_BUDGET，ACK 丢失。建议成功后清位，失败重置并限次重试。
- **C9（低）每拍 ACK 上限 2 实际可能只发 1**：循环按 word 递增，每个 word 只取一位；同一 word 内多个 CMD 下一拍才发。
- **C10（高）ACK 与事件不同步**：事件满时 `vision_emit` 丢弃，但 pending 仍清、ACK 仍发；视觉以为送达，mission 永远收不到。

### 2.4 pending 状态机

正确：32 位回绕均用 `(int32_t)(now-x) >= 0`；扫码/中位线/顺序/失联/取消逻辑完整。

问题：

- **C11 同拍竞态**：见 1.6。
- **C12（高）中位线无限重发且阻塞 TrackStart；无 CancelScan/CancelOrder**。`vision_link.c:853-871` 无 deadline；TrackStart 要求 pending NONE（`:1276-1279`）；`CancelMidline` 只取消中位线。mission 很容易卡死。建议 TrackStart 前自动取消中位线，或给中位线加总 deadline，并补取消接口。
- **C13（中）busy 规则不一致**：StartScan 检查 pending 或追踪；扫码/中位线/顺序只检查 pending，追踪中仍可发起，可能让视觉复位检测器打断追踪。
- **C14（低）TrackStop 未追踪时返回 OK，与头文件 ERR_STATE 描述不符**（`vision_link.h:147` vs `vision_link.c:1329-1332`）；未追踪时仍会多发一条 stop（无害）。

### 2.5 事件环

- 容量 8，实际可用 7；满时丢最新事件并计 event_overflow。
- 生产/消费都在任务上下文；若跨任务则无临界区保护。
- **C15（高）事件满时丢新事件且无优先级**；LINK_LOST/TXN_ABORTED/C_TARGET_DOWN 可能被丢，而 abort 仍清状态。
- **C16（高）`vision_emit` 失败但 pending 仍被清**（`vision_link.c:714-725,727-746,764-778,780-794`）：事件丢失后 mission 永远等不到结果，视觉还收到 ACK。
- **C17（中）事件环无消费者**：mission 接入前事件只会堆满并丢弃。

### 2.6 临界区

- 位置：`vision_link.c:595-598,633-644,669-678,905-907,957-971,1302-1307,1384-1389,1433-1448`。
- 屏蔽 USART3（优先级 5）与 TIM2（优先级 15）；临界区内无 HAL_GetTick、无发送、无长循环；最长是 GetStats 14 字段拷贝与 ACK 位图 8 word 清零，微秒级。
- **C17（中）优先级不变式未强制**：若 USART3 优先级改为 4 或 max syscall 改为 6，临界区不再屏蔽 ISR，出现真实竞态。建议 Init 中 `configASSERT` 并注明。
- **C2 二次临界区**：见 2.1。

### 2.7 ISR 预算

- 115200 约 86.8 us/字节；状态机 + 最多 4 字节入队 + 置 ACK 位为几十到几百周期，远小于预算。
- 50 Hz 流 400 B/s，200 Hz 上限 1600 B/s，CPU 占用可忽略。
- `HAL_GetTick` 在 ISR 中安全：uwTick 32 位对齐 volatile，单次 LDR，不会撕裂。
- 其他 UART ISR 解析均为短 switch，USART3 被饿死风险低。
- 结论：ISR 预算足够；保持 USART3 优先级不低于 5。

### 2.8 Init 注册与 uart_rx 槽位

- `App_Init` 在 `osKernelInitialize` 之前（`main.c:158/162`），`Vision_Link_Init` 在 `app_init.c:22`。
- 注册顺序：gyro→huart2、step→huart1、servo→huart4、vision→huart3；当前正好 4 个，USART3 不会注册失败。
- **C19（中）注册失败无任何提示**：`Uart_Rx_Register` 为 void，槽位满时静默返回；`Uart_Rx_Start` 与 `HAL_UART_Receive_IT` 返回值也被忽略。未来增加第 5 个 UART 时，USART3 可能拿不到槽位，但所有 API 只表现为 OFFLINE。建议返回状态 + `configASSERT`。

### 2.9 同任务上下文约束

- 头文件要求命令 API 与 Update 同任务（`vision_link.h:7-8`）；当前唯一调用者是 MainTask。
- 若 mission 独立任务调用：
  - `s_pending` 多字段非原子，Update 可能看到 kind 已改但 zone/tx_cmd 未改；
  - `s_track_active`、pose 三轴 + 标志同样非原子；
  - 事件环 head/tail 无临界区，两个任务 emit/take 会丢事件或索引错乱；
  - 两个任务同时 `HAL_UART_Transmit` 会随机 `HAL_BUSY`；
  - `s_stats` 跨任务读写竞态。
- 建议：mission 收敛到 MainTask，或给 vision_link 加互斥/命令队列；头文件应明确 TakeEvent/GetTrack/GetStats/ReportTcpPose 也受同任务约束。

### 2.10 统计、类型、未使用项

- `-Wall -Wextra` 语法检查无警告（注意是 `-fsyntax-only`，不是完整构建）。
- **C21（中）统计语义不足**：rx_frames 把非法帧也算入；err_noise 低估；缺少 ack_sent/ack_dropped/scan_timeouts/order_fallbacks/midline_retries/track_samples/event_drop_by_type 等现场排障计数；late_responses/err_unexpected 语义未注释。
- **C22（低）`Vision_Link_Ret_e` 定义但所有 API 返回 int**，类型安全丢失；`s_track_active`/`pending.zone` 用 uint8_t 存枚举；`vision_get_be16` 强转 int16 依赖 GCC 补码（正确）。
- **C25（低）未使用/死代码**：`s_show_scan` 从未置 1；`s_id_target` 只在不可达的 S19 使用；`s_last_path_ev` 只写不读；`CHASSIS_HEADING_WZ_FF` 定义未用。
- **C26/E2（中）构建产物一致性**：审查代理看到的 ELF 时间戳早于 `vision_link.c`，且 ELF 公共 API 被 GC。需重新完整构建并保存 map/ELF 作为联调基线。当前 ELF 的 Flash 58480 B、RAM( data+bss) 17020 B 与 HANDOVER 记录的 57988/15472 不一致，需复核。

---

## 3. 工程与文档一致性

### 3.1 文档与代码差异

| 文档 | 过时/不一致内容 | 证据 | 影响/建议 |
|---|---|---|---|
| HANDOVER | "视觉/上位机 USART3（协议已定，代码未接）" | `HANDOVER.md:41` vs `app_init.c:22`、`vision_link.c:1039-1090` | 已接；改为"vision_link 已接，未联调" |
| HANDOVER | "mission/vision 尚未实现" | `HANDOVER.md:50` | vision 已实现，mission 未实现；分开写 |
| HANDOVER | "视觉通信层 100%" | `HANDOVER.md:111` | 代码完成但未联调、无消费者、有协议缺口；改为"代码完成，未联调" |
| HANDOVER | "心跳与任意合法帧刷新链路时间" | `HANDOVER.md:173` vs `vision_link.c:368-374` | 实际是任意 XOR 正确帧，含非法载荷/未知 CMD |
| HANDOVER | 构建数字 RAM 23.6%（15472 B）、Flash 11.1%（57988 B） | `HANDOVER.md:181`；现有 ELF size 为 Flash 58480、RAM(data+bss) 17020 | 与现有 ELF 不符且 ELF 过期；重建后更新 |
| HANDOVER | 只列 B 的 0x20/0x21、0x29/0x2A | `HANDOVER.md:165-166` | 摘要不完整，补 C/D 对应命令 |
| DEBUG_NOTES | `MM_PER_PULSE` 写 0.002298 | `DEBUG_NOTES.md:16` vs `:185`、`chassis_config.h:45` | 同文档自相矛盾；代码用 0.002242 |
| DEBUG_NOTES | "旧版前馈 CHASSIS_HEADING_WZ_FF 已移除" | `DEBUG_NOTES.md:68` vs `chassis_config.h:113-114` | 宏仍定义但未使用；改为"保留未用" |
| DEBUG_NOTES | "完成后再跑按键 9 整条链" | `DEBUG_NOTES.md:190,213` | S14 不可达；S20 下按 9 是写 ID2 启动位置，会误写舵机参数 |
| ARM_DEBUG_NOTES | 参数表大量过时 | `ARM_DEBUG_NOTES.md:178-190` vs `arm_config.h` | 见 3.2 |
| ARM_DEBUG_NOTES | 测试关节角写 0/10/-10/0 | `ARM_DEBUG_NOTES.md:190` vs `:69`、`arm_config.h:139-142`（0/60/90/0） | 自相矛盾 |
| ARM_DEBUG_NOTES | "先在 S15/S16 按 9 回读" | `ARM_DEBUG_NOTES.md:210` | S15/S16 无入口，步骤无法执行 |
| CHASSIS_COMPARISON | 26GX 任务优先级写 MainTask02 AboveNormal、CarTask BelowNormal | `CHASSIS_COMPARISON.md:99` vs `main.c:198-200` | 实际 CarTask Realtime、EncTask AboveNormal、MainTask Normal |
| CHASSIS_COMPARISON | `CHASSIS_PATH_PERIOD_MS` 未使用 | `CHASSIS_COMPARISON.md:127` | 准确，保留 |
| 通信协议 README | v1.5.1 索引与不兼容点 | `include/通信协议_README.md:5-22` | 与当前文档一致 |

### 3.2 ARM_DEBUG_NOTES 参数表与代码对照

| 项 | 文档值 | 代码值 | 结论 |
|---|---|---|---|
| J2/J3/J4 零位 PWM | 1500/1500/1500 | 846/858/1559（`arm_config.h:39-41`） | 严重过时 |
| 连杆 L0~L3 | 0/100/100/50 | H0=55、上臂 120、偏置 20、小臂 160、TCP 80/55（`:78-84`） | 严重过时 |
| J2/J3/J4 限位 | ±180/±120/±120/±120 | 0..182/0..179/-89..79（`:101-106`） | 严重过时 |
| 工作空间 R/Z | 30/230、-50/250 | 30/400、-100/300（`:110-113`） | 过时 |
| 收拢姿态 | 0/0/0/0 | 0/181/178/78（`:117-120`） | 严重过时 |
| 测试关节角 | 0/10/-10/0 | 0/60/90/0（`:139-142`） | 过时且与文档 69 行矛盾 |
| 测试位姿 | 150/0/120/0 | 150/0/120/0（`:133-136`） | 一致 |
| 夹爪 | 833/1278/1500 | 833/1278/1500（`:64-66`） | 一致 |
| PTP 上限/容差 | 90/120、2.0 | 90/120、2.0（`:123-126`） | 一致 |

建议：参数表直接从 `arm_config.h` 重新生成，避免现场按旧值标定。

### 3.3 app_debug 的现场影响

| 问题 | 证据 | 影响 |
|---|---|---|
| 默认页 S20 | `app_debug.c:23` | 上电看不到底盘/视觉 |
| 长按 3 只在 S20/S18 切页 | `app_debug.c:227-253` | S15/S16/S17/S19/S14 均无入口 |
| S14 路径页无入口 | `app_debug.c:512-547`；切页只在 `:244` | 无法从键盘跑路径；DEBUG_NOTES 的"按 9 跑整条链"实际在 S20 写 ID2 启动位置（`:502-505`） |
| S15/S16/S17 无入口 | 处理代码 `:337-386`，切页只到 S18/S20 | J1 点动、舵机点动、夹爪/激光测试无法执行 |
| S19 假提示 "LONG3 NEXTPAGE" | `app_debug.c:1188`；切页只覆盖 S20/S18 | 提示无效 |
| `s_show_scan` 从未置 1 | `:59`、`:471-503`、`:933,1192` | 扫描页与 `Arm_Debug_ServoScan*` 全部不可达 |
| `s_last_path_ev` 只写不读 | `:152,167` | 调试残留 |
| 长按 3 的副作用 | S20 下先广播恢复扭矩（`:486-490`），1.2 s 后才切 S18 | 进 S18 同时发一次 `#255PULR!`，通常无害 |
| 无视觉调试页 | 全工程无 `Vision_Link_GetStats` 调用 | 现场看不到在线状态、收发计数、超时次数 |

建议：恢复 S14 入口或改默认页；修复 S19 提示；补 `s_show_scan` 入口或删除死代码；增加视觉统计页；更新按键文档。

### 3.4 include、构建与重复符号

- `build_src_filter` 为 `+<../Core/Src/*.c>`、`+<**/*.c>`（`platformio.ini:19-21`）；`src/app/vision_link.c` 已生成对象文件。
- include 路径完整（`platformio.ini:23-29`）。
- 重复符号检查：`HAL_UART_RxCpltCallback`、`HAL_UART_ErrorCallback`、`HAL_TIM_PeriodElapsedCallback` 各一处；`huart3` 只在 `main.c:53` 定义，`vision_link.c:19` 为 extern。
- `stm32f1xx_it.c:60,65` 有重复的 `extern huart2` 声明，不产生符号，建议清理。
- `Middlewares/` 不参与编译，无重复内核。
- **最大工程隐患**：`Uart_Rx_Register/Start` 静默失败（见 2.8）。

### 3.5 TODO/占位归类

| 子系统 | 位置 | 内容 |
|---|---|---|
| 视觉/任务 | `HANDOVER.md:50,111-113,177` | mission 未实现、vision 未联调、A/B/C/D 仍 500 ms 模拟 |
| 视觉/任务 | `app_tasks.c:66-68` | 上电 3.5 s 校准期间 Vision_Link_Update 不运行 |
| 视觉/任务 | `app_debug.c:147-148,173` | 到达任务点 500 ms 自动放行 |
| 视觉/任务 | `app_debug.c:23,59,1188` | S20 默认、死代码、假提示 |
| 底盘 | `path_table.c:9` | 距离、扫描长度、超时均为占位 |
| 底盘 | `chassis_config.h:13,58,191` | 机械/传动、速度/加速度、WIT 寄存器 TODO |
| 底盘 | `chassis_config.h:114` | `CHASSIS_HEADING_WZ_FF` 未使用 |
| 底盘 | `ctrl.c:198` | `osMessageQueuePut` 返回值忽略 |
| 底盘 | `path_table.c:16-25`、`path.c:264-266` | A 点无 PATH_TASK/ARRIVE |
| 机械臂 | `arm_config.h:16,18,29,54,109` | 细分、速度、J2 ID、半双工极性、限位 TODO |
| 机械臂 | `arm_kin.c:10` | 假设 q3=0 时小臂与大臂同向 |
| 机械臂 | `arm_config.h:71`、`laser.c:26-33` | 激光 GPIO 未分配，操作为空 |
| 机械臂 | `HANDOVER.md:249-252` | tool_deg 未提供；IK 偏置/TCP 未验证；夹爪 270° 假设 |
| 机械臂 | `ARM_DEBUG_NOTES.md:168-191` | 参数表待填且大量与 config 不符 |
| 机械臂 | `ARM_PLAN_REVIEW.md:17,37-46,104,129` | 动力学/PTP S 曲线/回读校正/示教未实现 |
| 工程 | `HANDOVER.md:183` | 无 git，改动不可回滚 |
| 工程 | `test/README` | 无测试 |

## 4. 实现与协议不一致清单（需拍板）

1. **0x14 方向/端点**：协议表写 MCU→上位机，§5.3 又写视觉只记录；实现发到 USART3。需确认上位机是否同一 USART3、0x14 是否要 ACK。
2. **ACK 语义**：每帧一个还是每 CMD 一个？畸形帧是否回 ACK？ACK 表示"收到"还是"已处理"？当前位图合并且可能先于事件入队。
3. **扫码"1 s×3 重发"**：当前总共 3 次；若"重发"指首帧后 3 次，应为 4 次。
4. **中位线 0/2/3 后是否自动继续请求**：当前收到任何值都清 pending；协议说反复请求直到成功。
5. **"连续 5 次"丢失定义**：当前 5 个 500 ms 无样本窗口；协议可能指 5 次 0x13/5 个失稳段。
6. **0x12 事务关联**：错误码无请求 ID，当前靠 pending 类型猜；需定义各事务遇到哪些码清事务、哪些可重试。
7. **MCU→视觉命令送达/重发**：0x28/0x22/0x24/0x25/0x27/0x34 无确认，当前只发一次。
8. **0x22→0x25 目标切换**：当前要求先 Stop，协议未写。
9. **0x34→开激光、0x35/3 s→关激光**：谁负责、3 s 从何时起算、激光 GPIO 未分配。
10. **0x2A 未检出=1 与中位=1 不可区分**：建议视觉改发 0x12(0x04) 或加标志。
11. **"回展开态重粗调"具体动作**：展开姿态、谁调用、如何恢复追踪均未定义。
12. **在线判定口径**：是否包括未知 CMD/非法载荷/错误方向帧？当前全部刷新。
13. **心跳 LEN≠0 是否算在线**：当前仍刷新。
14. **0x13/TRACK_LOST 的 target 标识**：事件只有 zone，B 物体/B 桶无法区分。
15. **0x21=0（未检出）后 MCU 行为**：继续请求、移动底盘还是报错？
16. **0x12 未来错误码 0x07+/0x00**：当前判非法载荷、不回 ACK；视觉扩展错误码会被静默丢弃。
17. **0x11 是否期待 ACK**：协议未豁免，实现会 ACK。
18. **0x14/0xFF 在 ACK 豁免表中的冗余**：它们是 MCU 发出的，MCU 正常收不到。
19. **hz 上限 200、0x14 限速 50 Hz**：均为实现自设，协议无数值。
20. **B 物体/B 桶偏移是否允许交错**：当前非活动目标的偏移会写槽位但不被使用。

## 5. 待补充现场数据/业务规则

1. 视觉端心跳周期/相位、追踪流实际频率、帧间最大间隔与突发长度。
2. 视觉端是否对 MCU→视觉命令做确认/重发；丢失时的现场表现。
3. 各任务 0x12 错误码实际使用：哪些致命、哪些瞬态；0x12 与请求的对应关系。
4. 中位线判定耗时、0/2/3 分布、从 2/3 到 1 的底盘调整量、500 ms 重发是否够快。
5. 顺序判定延迟、3 s 是否足够、未检出时视觉回 0x2A=1 还是 0x12(0x04)。
6. 追踪流频率/丢帧率、0x13 发送时机、5X 现场定义、5X 后期望动作与恢复时间。
7. 0x14 的实际目的地、期望频率、是否需记录/回放、z 坐标系。
8. 0x34 后开激光、0x35/3 s 后关激光的准确时序；激光 GPIO；3 s 起算点。
9. 工具系标定：`R_tool→base`、x/y/z 实际方向、dx/dy 符号、相机安装偏差；预设 z/接近策略。
10. 0x2A 名次 0/1/2 与 B/C/D 预设位置对应表。
11. B 物体→B 桶切换时序、0x25 的 Hz、桶放置机械臂动作。
12. 失联时期望整车行为：底盘停、机械臂 Stow、是否自动恢复、从哪一步继续。
13. 事件消费策略：每拍 drain 还是按需、哪些事件必须不丢、队列容量。
14. 实际最大突发帧数，用于确认 RX 队列 8、事件队列 8 是否足够。
15. USART3 优先级/FreeRTOS 配置能否改动；能否保持优先级 5。
16. 现场线路噪声/误码率；XOR 是否足够；是否需 CRC；0xA5 在载荷中的出现概率。
17. 视觉端是否与上位机共用 USART3；0x14 是否由视觉转发。
18. 0x23/0x26 是否可能交错；0x13 是否按 target 区分。
19. 0x21=0 时 mission 期望动作（继续扫描/移动/报错）。
20. 0x12 未来错误码规划；0x11 是否需要 ACK。

## 6. 必须修代码才能联调（按严重度）

### S0：联调前必须补的模块/入口

1. **mission 缺失，事件无消费者**。全工程无 `Vision_Link_TakeEvent` 调用；ELF 中公共 API 被 GC。A/B/C/D 无法执行，事件堆满后丢弃。建议先写最小 mission，同 MainTask、每 5 ms drain。
2. **S14/S15/S16/S17 无入口，DEBUG_NOTES 的按键 9 会误写 ID2 启动位置**。建议恢复入口或改默认页，并修正文档。

### S1：会导致任务卡死/状态分叉/安全动作丢失

3. **0x34/0x35 激光动作 + 3 s 超时未实现**；C 任务无法完成。建议 mission 在 TrackStop 后开激光，在 C_TARGET_DOWN/3 s 后关激光；分配 GPIO。
4. **MCU→视觉 stop/start 无送达确认**（0x24/0x27/0x28/0x34）。stop 丢失会让视觉继续推流 60 s。需协议或 MCU 侧重发策略。
5. **同目标 TrackStart 不重发，5X 后重启无效**。建议 5X 清 active 或同目标强制重发。
6. **中位线无限重发且阻塞 TrackStart；无 CancelScan/CancelOrder**。建议 TrackStart 前自动 CancelMidline，或给中位线加总 deadline，并补取消接口。
7. **0x12 不关联 midline/order pending**，致命错误会无限重试。建议按 pending 类型清事务并带错误事件。
8. **响应与超时同拍竞态**。建议 pending 超时检查前再 drain 一次，或检查 RX 队列非空。
9. **事件环满时丢事件但 pending 仍清、ACK 仍发**。建议 `vision_emit` 返回状态，仅成功入队才清 pending/ACK；关键事件用 latched 标志。
10. **ACK 先清位再发送，失败/无预算时永久丢失**。建议成功后清位，失败重置并限次重试。
11. **链路在线判定过宽**（任意 XOR 正确帧都刷新）。建议只在合法命令/载荷/心跳后刷新，或单独维护 `last_valid_ms`。
12. **同任务约束未强制**，mission 独立任务会撕裂 pending/事件/pose。建议收敛到 MainTask，或加 mutex/命令队列。

### S2：高概率现场问题/一致性缺陷

13. **0x2A 未检出=1 与中位=1 不可区分**，可能选错目标。
14. **构建产物过期/数字不一致**。先完整 `pio run`，保存新 ELF/map 作为基线。
15. **S19 假提示、`s_show_scan` 死代码、S15/S16 文档不可执行**。
16. **TrackStop 未追踪时返回 OK，与头文件 ERR_STATE 描述不符**。统一语义。
17. **USART3 优先级不变式未强制 + uart_rx 静默失败**。加 `configASSERT`，让注册返回状态。
18. **`HAL_UART_Transmit` 阻塞 20 ms 在 MainTask；超时可能截断帧**。建议中断/DMA 发送。
19. **同 CMD ACK 合并、每拍上限 2 实际可能只 1**。确认 ACK 语义后修正。

### S3：中低优先级/代码质量

20. 统计字段不足/语义不清（C21）。
21. 解析器残帧超时二次临界区、截断帧吞 A5、噪声低估（C1/C2/C3）。
22. 事件无 target 字段（P10）。
23. 0x14/0xFF 豁免表冗余（P24）。
24. 心跳 LEN≠0 仍在线（P9）。
25. `s_last_path_ev` 写-only、`CHASSIS_HEADING_WZ_FF` 死宏（C25）。
26. 无静态断言保证队列长度是 2 的幂（C6）。
27. 无单元测试；建议为解析器/XOR/队列/pending 加 host 测试。
28. 文档批量过时（3.1/3.2）。

## 7. 联调顺序建议

1. 修文档/调试页：恢复 S14 或改默认页；修正 DEBUG_NOTES 的"按键 9"；更新 HANDOVER。
2. 完整构建并保存新 ELF，确认公共 API 不再被 GC。
3. 写最小 mission：同 MainTask、每 5 ms drain；先做 A 扫码 + B 扫描/中位线/顺序，验证 ACK、超时、事件。
4. 修 S1 的 3~12 项：激光、stop 重发、中位线取消、0x12 关联、同拍 drain、事件/ACK 顺序、在线判定、同任务约束。
5. 联调追踪：50 Hz、dx/dy 符号、5X、同目标重启、B 物体→B 桶切换。
6. 接 C/D 与异常：激光 3 s、失联回展开、60 s 超时、噪声/断线恢复。

---

# 四、合并结论与补齐路线

## 1. 全局判断

三份审查的结论一致：

- **驱动层成熟**：电机、编码器、陀螺、步进、舵机、夹爪、视觉帧层都能工作。
- **单段能力可用**：底盘单段运动、机械臂单动作、视觉单帧收发都能独立验证。
- **逻辑链接缺失**：没有 mission，路径事件仍由 debug 的 500 ms 假完成消费，视觉事件无人消费，机械臂正式动作无调用方。
- **当前烧录不会自主跑图**：默认 S20、AUTO=0、没有可用路径入口；强行启动也会卡在 A 点和 B/C/D 的 `SEG_TASK_WAIT`。

一句话：**不是"某一块没写完"，而是"三块能力之间没有编排层和契约"。**

## 2. 差距分类

### A. 代码缺失（阻塞落地）

| # | 缺口 | 现有证据 | 影响 |
|---|---|---|---|
| A1 | mission 任务层不存在 | `src/app` 无 mission 文件 | 无任何任务编排 |
| A2 | 视觉事件/命令 API 零消费者 | `vision_link.h:127-160`，全工程无调用 | 视觉只是"通电" |
| A3 | A 点无到达事件 | `path_table.c:17-26` | 任务链卡在 A |
| A4 | 无扫描段开始事件 | `path.c:285-307` 只在扫描结束发 SCAN_READY | 无法在正确时机发 0x28/0x20/0x29 |
| A5 | 机械臂正式动作无调用方、无完成事件 | `arm.c:106-151` | B/C/D 动作无法编排 |
| A6 | 无按 order 选预设位/底盘横向微调 API | `chassis.h:32-39` 只有路径/停止 | 0x2A 结果无法使用；中位线 2/3 无法修正 |
| A7 | 无 dx/dy → 机械臂流式微调接口 | PTP 最小 200 ms、单命令 BUSY | 50 Hz 追踪闭环无法实现 |
| A8 | 0x14 缺 FK/TCP 位姿公开 API | `arm_kin.h:17`、`joint_fb.h:18` 均在内部层 | 无法上报位姿 |
| A9 | 激光开关与 3 s 超时无实现 | `ARM_LASER_ENABLE=0`、`laser.c` 空操作 | C 段不可执行 |
| A10 | 无路径运行时入口/中止/急停 | S3~S14 不可达；`Path_Abort`/`Control_Stop` 零调用 | 现场无法启动或安全停止 |
| A11 | 路径 fault 不传播、TASK_WAIT 无超时 | `path.c:454-468,496-504` | 故障被当成功；永久卡死 |
| A12 | 路径事件单槽 | `path.c:48,106-111` | 事件覆盖丢失 |
| A13 | Ctrl Submit 忽略队列满、无回滚 | `ctrl.c:185-199` | 命令丢失、航向漂移 |
| A14 | MainTask 512 B 栈 | `main.c:64-68` | 接入 mission 后溢出风险 |
| A15 | vision_link 5 项 S1 缺陷 | 见第三节 6.S1 | 追踪重启无效、事务卡死、ACK 丢失 |

### B. 实现缺陷（可靠性/一致性）

| # | 缺陷 | 证据 | 影响 |
|---|---|---|---|
| B1 | 调试命令 `Chassis_Cmd_t` 未初始化 | `ctrl.c:787-861` | 调试运动末端行为随机 |
| B2 | `end_speed` 非零仍停车 | `ctrl.c:392-395,655-667` | "段间平滑"不成立 |
| B3 | stale `finished` + seq 复用 | `ctrl.c:245-260,863-877`、`path.c:376-384` | 可能误判段完成 |
| B4 | 调试 pulse hold 永久保持 override | `chassis.c:152-196` | 后续正式运动被跳过 |
| B5 | J1 软件位置假回读 | `arm_motion.c:222-225`、`joint_fb.c:156-160` | 丢步/复位无法发现 |
| B6 | PTP 稳定 3 次实为 3 个控制拍 | `arm_motion.c:297-307` | 未真正稳定 |
| B7 | Pose/Aim 不检查工作空间 | `arm_action.c:268-278,340-350` | 越界目标可能被接受 |
| B8 | Pick&Place 夹爪无等待、无每步超时 | `arm_action.c:114-202` | 未夹住/未松开就抬臂；8 s 易超时 |
| B9 | Aim 激光不可用仍成功、无自动关 | `arm_actuator.c:48-53`、`arm_action.c:398-403` | C 段假成功、激光常亮 |
| B10 | `Arm_Stop()` 清错误、无故障锁存 | `arm_action.c:437-446` | 急停与故障不可区分 |
| B11 | `s_servo_target` 与正式 PTP 不同步 | `arm.c:24,59-61,192-224` | S16 点动可能大幅跳动 |
| B12 | NaN 通过限位、`Wrap180` 对 Inf 死循环 | `arm_limit.c:20-35`、`math_util.c:11-19` | 未定义行为 |
| B13 | `dbg_stpcpy` 无长度参数 | `app_debug.c:694-702` | 未来新增字符串可能越界 |
| B14 | 完成队列无人消费 | `ctrl.c:262-265,713-733` | 死接口 |
| B15 | `CHASSIS_PATH_PERIOD_MS` 未使用 | `chassis_config.h:88` | 改周期时易踩坑 |

### C. 参数未标定

| # | 参数 | 证据 | 必须动作 |
|---|---|---|---|
| C1 | `MM_PER_PULSE` 三套口径 | 0.003324 理论 / 0.002242 代码 / 0.002298 文档 | 3000~5000 mm 复标 |
| C2 | 航向 KI | `chassis_config.h:104,109` | 按 CHASSIS_COMPARISON 的 7 步验证 |
| C3 | 圆弧补偿单工作点 | `chassis_config.h:148-155` | 换半径/速度必须重标 |
| C4 | 轮距 190 无侧滑修正 | `odom.c:81-82` | 标定有效轮距或降融合权重 |
| C5 | 路径表距离/扫描长度 | `path_table.c:9` | 地图实测 |
| C6 | 机械臂连杆/零位/限位/收拢 | `arm_config.h:80-120` | 三姿态实车验证 |
| C7 | 20 mm 偏置符号/TCP/q3 定义 | `arm_config.h:89-92`、`arm_kin.c:5-10` | 零位测 TCP 确认 |
| C8 | J1 细分/速度/加速度/极性 | `arm_config.h:16-19,54-56` | 按驱动器菜单与负载整定 |
| C9 | PTP 参数 | `arm_config.h:122-130` | 测到位误差/超调/稳定时间 |
| C10 | 夹爪 180/270 | `arm_config.h:58-66` | 实测行程 |
| C11 | 工作空间 z 上限与文档冲突 | `arm_config.h:110-113` vs `ARM_DEBUG_NOTES.md:95` | 统一并实测 |
| C12 | `tool_deg` 预设姿态 | `arm_types.h:39`、`HANDOVER.md:249` | 主理人提供 |
| C13 | 激光 GPIO | `arm_config.h:71` | 主理人提供 |

### D. 业务规则未定义

1. **取图时机**：`SCAN_READY`（扫描结束、300 ms 后倒回）还是 `ARRIVE_POINT`（倒回中心）？前者相机在动，后者需要任务层最终通知。
2. **扫码失败策略**：0x12 错误码 1..6 各自是重试、跳过还是中止整条链？0x11 后是否立即发车？
3. **中位线 0/2/3**：收到 2/3 后是原地微调再请求、前进后重试，还是忽略？底盘横向微调能力未定义。
4. **顺序名次**：0/1/2 与 B/C/D 预设位置的对应表；超时兜底为 1 后是否仍前进。
5. **追踪 5 次丢失后**："回展开态"具体是 Stow、Home 还是自定义姿态？多久后重试？谁执行？
6. **失联时整车行为**：底盘停、机械臂 Stow、激光关？是否允许自动恢复？恢复后从哪一步继续？
7. **激光时序**：0x34 后立刻开？0x35 或 3 s 从何时起算？Aim 成功是否等同于开激光？
8. **任务完成判定**：视觉成功、机械臂成功还是超时降级？失败时重试/跳过/中止？
9. **急停策略**：一个键同时停底盘/机械臂/激光/夹爪？急停后能否自动恢复？
10. **机械臂-底盘互锁**：行车前必须 Stow？机械臂动作时底盘必须停？
11. **B 物体→B 桶切换**：是否必须先 0x24 再 0x25？桶追踪的成功判据？
12. **A 点**：停车扫码还是仅途经？扫描位置与朝向？

### E. 仅缺现场数据

地图距离、视觉延迟/帧率/丢帧、300 ms 停留是否够、地面摩擦/履带差异、夹爪真实行程、激光 GPIO、齿隙/重力下垂/重复定位、5 分钟分段耗时、整车尺寸与收拢碰撞。

## 3. 不明确点（需主理人拍板）

### 协议层

1. 0x14 端点：上位机是否同一 USART3？是否要 ACK？
2. ACK：每帧一个还是每 CMD 一个？畸形帧是否回 ACK？ACK 表示"收到"还是"已处理"？
3. 扫码"1 s×3"：总共 3 次还是首发 + 3 次重发？
4. 中位线 0/2/3 后是否由通信层自动继续请求？
5. "连续 5 次丢失"：5 个 500 ms 窗口还是 5 次 0x13/5 个失稳段？
6. 0x12 事务关联：各事务遇到哪些错误码清 pending、哪些可重试？
7. MCU→视觉命令（0x28/0x22/0x24/0x25/0x27/0x34）是否需要送达确认/重发？
8. 0x2A 未检出回 1 与名次 1 如何区分？是否改发 0x12(0x04)？
9. 在线判定是否包括未知 CMD/非法载荷/错误方向帧？
10. 0x13/TRACK_LOST 是否增加 target 字段？
11. 0x21=0 时 mission 的动作？
12. 0x12 未来错误码扩展策略；0x11 是否期待 ACK。
13. hz 上限、0x14 频率是否有协议约束。
14. B 物体/B 桶偏移是否允许交错。

### 业务层

15. 任务失败与重试策略（扫码/中位线/顺序/追踪/机械臂动作）。
16. 失联/急停后的恢复流程。
17. 视觉主控还是 MCU 主控任务节奏？当前协议是 MCU 请求、视觉应答。
18. 现场是否允许人工干预（按键接管、手动对正 J1）？何时禁止。

## 4. 建议实施顺序（唯一路线）

### 阶段 0：不依赖实车的代码修复（约 1~2 天）

目标：让现有能力可诊断、可安全停止、可完整构建。

1. 修 `ctrl.c` 三个调试命令未初始化。
2. 修或移除 `end_speed` 段间平滑：要么实现"到边界不停车"，要么删除说法与参数。
3. 路径层：检查 `fault`、`TASK_WAIT` 超时、事件改 4~8 深队列、`NotifyTaskDone(0)` 语义。
4. `Ctrl_Submit` 返回失败、入队成功后再累加航向；提交新命令清 `finished`。
5. 表 0 增加 `PATH_TASK`（`point_id='A'`）。
6. 恢复 S14 入口；增加真正的路径中止键与 `Chassis_Control_Stop()` 入口。
7. 修正 DEBUG_NOTES"按键 9"风险；更新 ARM_DEBUG_NOTES 参数表与 HANDOVER。
8. vision_link S1 修复：
   - 5X 清 active 或同目标强制重发 0x22；
   - 0x12 按 pending 类型清事务；
   - `vision_emit` 返回状态，仅成功入队才清 pending/ACK；
   - ACK 发送成功后清位，失败重置；
   - 链路时间只在合法帧后刷新；
   - TrackStop 语义与头文件统一。
9. MainTask 栈提升到 1 KB；OLED 或统计接口显示高水位。
10. 完整 `pio run`，保存 ELF/map 作为基线。

### 阶段 1：mission 最小闭环（约 3~5 天）

目标：A 点扫码 + B 点扫描/中位线/顺序跑通，验证视觉链路与路径事件契约。

1. 新增 `app/mission`，放在 MainTask，同上下文调用 `Vision_Link_Update`。
2. mission 独占路径事件消费；`App_Debug_HandlePathEvents` 的 500 ms 模拟用编译开关隔离或删除。
3. 实现 A 点：`ARRIVE('A')` → `RequestScanCode` → 事件 → `NotifyTaskDone('A')`。
4. 实现 B 点最小流程：扫描开始通知 → `StartScan(B)` → `RequestMidline(B)` → `RequestOrder(B)` → 固定名次 1 前进 → `NotifyTaskDone('B')`。
5. 定义并实现：取图时机、扫码失败策略、中位线 0/2/3 处理、顺序兜底。
6. 增加底盘"按名次到预设位"的最小接口（可先只支持名次 1）。
7. 验收：串口助手/视觉端联调，ACK、重发、失联只报一次、事件不丢。

### 阶段 2：机械臂实车标定与接口（约 3~5 天）

目标：让机械臂具备任务级可用的位姿与反馈。

1. 三姿态标定：零位 TCP、q3=+90、q4=−90；确认偏置符号、TCP 坐标系、肘部。
2. J1 真实回读接入完成判定；识别步进复位/离线。
3. `arm.h` 增加 FK/TCP 位姿查询、关节角查询。
4. 增加 50 Hz 流式微调接口（小步 PTP 或专用增量接口），明确 BUSY 重试契约。
5. 夹爪加 `ARM_GRIPPER_MOVE_MS` 等待；S17 增加 GRASP 测试键并恢复入口。
6. 激光 GPIO 分配、`ARM_LASER_ENABLE=1`、Aim 可用性检查与自动关策略。
7. 写入 B/C/D 固定 `tool_deg` 与预设位姿。

### 阶段 3：B/C/D 任务编排（约 5~7 天）

1. B：排爆桶追踪 0x25→0x26→0x27 + Pick&Place + 0x22 物体追踪。
2. C：Aim + 0x34 停追踪开激光 + 0x35/3 s 关激光。
3. D：追踪 + 抓取/放置 + 0x44。
4. 失联/5X 回展开态、任务失败重试/跳过的业务实现。
5. 底盘-机械臂互锁：行车前 Stow、动作时停车。
6. 追踪 dx/dy → 基座坐标 → 机械臂微调闭环。

### 阶段 4：整链联调与比赛验收（约 3~5 天）

1. A→B→C→D→终点串行，5 分钟节拍。
2. 异常场景：目标丢失、通信失联、步进复位、舵机离线、急停恢复。
3. 路径表距离复标、扫描停留调整、速度档位确认。
4. 整车 250×250×300 mm 与收拢姿态碰撞检查。
5. 现场激光加工底盘后的重标定。

## 5. 里程碑验收标准

| 阶段 | 通过标准 |
|---|---|
| 0 | 构建无警告；S14 可进入并可中止；路径事件不丢；vision_link 单测/串口注入通过 |
| 1 | A 点扫码成功进入 B；B 点中位线/顺序返回并驱动底盘；失联只报一次；事件无溢出 |
| 2 | 三姿态 TCP 误差可解释；J1 丢步能报错；FK/IK 互查通过；激光可控 |
| 3 | B/C/D 各自独立完成；追踪微调方向正确；失败可重试或安全中止 |
| 4 | 全程 ≤5 分钟；异常不撞车、不停死；尺寸合规 |

## 6. 结论

工程不是"缺少某个功能"，而是**缺少将已有能力连接起来的四类契约**：

1. **事件契约**：路径事件多消费者、单槽覆盖、无超时；视觉事件无消费者。
2. **完成契约**：`NotifyTaskDone` 过弱、机械臂无完成事件、任务失败无定义。
3. **运动契约**：段间平滑名不副实、追踪微调无接口、按名次选位无接口。
4. **安全契约**：fault 不传播、失联无动作、无急停/中止、无故障锁存、栈余量不足。

先补契约（阶段 0~1），再补精度（阶段 2），最后补业务（阶段 3~4），比先做机械臂任务动作或先调航向更稳妥。
