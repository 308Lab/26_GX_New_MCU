/**
  ******************************************************************************
  * @file    app/vision_link_config.h
  * @brief   视觉链路唯一参数出口（协议 v1.6）。
  *
  * 本文件只允许 .c 包含。USART3 PB10/PB11 115200 8N1 由 CubeMX 配置，
  * 此处不重复定义波特率。
  ******************************************************************************
  */

#ifndef APP_VISION_LINK_CONFIG_H
#define APP_VISION_LINK_CONFIG_H

/* ===================== 1. 帧与队列 ===================== */
/* 视觉->MCU 最大载荷：v1.6 §5.1 的 0x11 扫码结果 ok,d1,d2,d3（4B） */
#define VISION_LINK_MAX_RX_DATA              (4U)
/* MCU->视觉 最大载荷：v1.6 §4.3 的 sel（1B）；留 1B 余量，帧长上限不变 */
#define VISION_LINK_MAX_TX_DATA              (2U)
/* 低频帧环形队列，必须是 2 的幂 */
#define VISION_LINK_RX_QUEUE_LEN             (8U)
/* 事件环形队列，必须是 2 的幂 */
#define VISION_LINK_EVENT_QUEUE_LEN          (16U)

/* ===================== 2. 链路时序 ===================== */
/* 协议 §1：连续 500ms 收不到视觉任何帧判失联 */
#define VISION_LINK_LINK_TIMEOUT_MS          (500U)
/* 残帧复位：115200 下一帧 <1ms，100ms 仅作异常兜底 */
#define VISION_LINK_FRAME_TIMEOUT_MS         (100U)
/* 审计 H5：USART3 发送改 IT + 环形缓冲，任务侧不再等 UART 发完（原 20ms 阻塞超时已删除）。
   下面这条只作「IT 停滞」兜底：一段 ≤6B@115200 8N1 ≈0.52ms，100ms 是 190 倍余量 */
#define VISION_LINK_TX_STALL_MS              (100U)
/* 发送环形缓冲（必须是 2 的幂，取模靠掩码）：整帧拷贝入队，只有放得下整帧才入队，
   保证帧不截断、不交叉。head == tail 表示空，故实际可用 63B；最长帧 = 头/码/长/数据/校验
   = 4 + VISION_LINK_MAX_TX_DATA(2) = 6B → 至少 10 帧，远大于单拍预算 4+2 帧 */
#define VISION_LINK_TX_RING_SIZE             (64U)
#if ((VISION_LINK_TX_RING_SIZE == 0U) \
    || ((VISION_LINK_TX_RING_SIZE & (VISION_LINK_TX_RING_SIZE - 1U)) != 0U))
#error "VISION_LINK_TX_RING_SIZE 必须是 2 的幂（发送环形缓冲用掩码取模）"
#endif
/* 每拍最多发送帧数（ACK + 重发 + 位姿）；改 IT 后此预算只限速，不再决定阻塞时间 */
#define VISION_LINK_TX_MAX_FRAMES_PER_UPDATE (4U)
/* 每拍最多 ACK 数：5B@115200 约 0.43ms */
#define VISION_LINK_ACK_MAX_PER_UPDATE       (2U)

/* ===================== 3. 请求-应答事务（协议 §6） ===================== */
/* 扫码：只发送 1 次，不重发 */
#define VISION_LINK_SCAN_RETRY_MS            (1000U)   /* MAX_TX=1 时不生效，保留定义 */
#define VISION_LINK_SCAN_MAX_TX              (1U)
/* 扫码：发出后 3s 仍无响应判失败 */
#define VISION_LINK_SCAN_RESPONSE_TIMEOUT_MS (3000U)
/* v1.6 §5.3 的「未检出」哨兵值定义在 vision_link.h（业务层也要用它判未检出） */
/* 中位线：500ms 无回包重发，可反复请求 */
#define VISION_LINK_MIDLINE_RETRY_MS         (500U)
/* 审计 H6：中位线 pending 生命周期上限。上位机侧 midline_reply_ttl=500ms 且以同一周期
   重发 0x20，MCU 侧超时必须明显长于该周期，否则会把「上位机正在重试」误判成失败；
   取 4 个重发窗口 = 2000ms，同时短于 mission 的 5s 中位线总超时，
   保证「没人取消」时 pending 也会自己结束，不把整条视觉链路永久堵成 ERR_BUSY。 */
#define VISION_LINK_MIDLINE_TIMEOUT_MS       (2000U)
/* 顺序判定：3s 无回包按 1=中兜底，不停车 */
#define VISION_LINK_ORDER_TIMEOUT_MS         (3000U)
#define VISION_LINK_ORDER_FALLBACK_VALUE     (1U)

/* ===================== 5. 协议命令号（v1.5.1） ===================== */
#define VISION_CMD_HEARTBEAT                 (0xFEU)
#define VISION_CMD_ACK                       (0xFFU)

/* A 点扫码 */
#define VISION_CMD_SCAN_CODE_REQ             (0x10U)
#define VISION_CMD_SCAN_CODE_OK              (0x11U)
#define VISION_CMD_ERROR                     (0x12U)
#define VISION_CMD_TARGET_LOST               (0x13U)

/* B：扫描 / 顺序 / 中位线 / 追踪 / 桶追踪 */
#define VISION_CMD_B_SCAN                    (0x28U)
#define VISION_CMD_B_ORDER_REQ               (0x29U)
#define VISION_CMD_B_ORDER_RSP               (0x2AU)
#define VISION_CMD_B_MIDLINE_REQ             (0x20U)
#define VISION_CMD_B_MIDLINE_RSP             (0x21U)
#define VISION_CMD_B_TRACK_START             (0x22U)
#define VISION_CMD_B_TRACK_OFFSET            (0x23U)
#define VISION_CMD_B_TRACK_STOP              (0x24U)
#define VISION_CMD_B_BUCKET_START            (0x25U)
#define VISION_CMD_B_BUCKET_OFFSET           (0x26U)
#define VISION_CMD_B_BUCKET_STOP             (0x27U)

/* C */
#define VISION_CMD_C_SCAN                    (0x38U)
#define VISION_CMD_C_ORDER_REQ               (0x39U)
#define VISION_CMD_C_ORDER_RSP               (0x3AU)
#define VISION_CMD_C_MIDLINE_REQ             (0x30U)
#define VISION_CMD_C_MIDLINE_RSP             (0x31U)
#define VISION_CMD_C_TRACK_START             (0x32U)
#define VISION_CMD_C_TRACK_OFFSET            (0x33U)
#define VISION_CMD_C_TRACK_STOP              (0x34U)
/* 0x35 视觉->MCU：C 目标倒下 */
#define VISION_CMD_C_TARGET_DOWN             (0x35U)

/* D */
#define VISION_CMD_D_SCAN                    (0x48U)
#define VISION_CMD_D_ORDER_REQ               (0x49U)
#define VISION_CMD_D_ORDER_RSP               (0x4AU)
#define VISION_CMD_D_MIDLINE_REQ             (0x40U)
#define VISION_CMD_D_MIDLINE_RSP             (0x41U)
#define VISION_CMD_D_TRACK_START             (0x42U)
#define VISION_CMD_D_TRACK_OFFSET            (0x43U)
#define VISION_CMD_D_TRACK_STOP              (0x44U)

#endif /* APP_VISION_LINK_CONFIG_H */
