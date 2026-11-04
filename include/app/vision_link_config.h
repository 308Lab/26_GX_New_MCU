/**
  ******************************************************************************
  * @file    app/vision_link_config.h
  * @brief   视觉链路唯一参数出口（协议 v1.5.1）。
  *
  * 本文件只允许 .c 包含。USART3 PB10/PB11 115200 8N1 由 CubeMX 配置，
  * 此处不重复定义波特率。
  ******************************************************************************
  */

#ifndef APP_VISION_LINK_CONFIG_H
#define APP_VISION_LINK_CONFIG_H

/* ===================== 1. 帧与队列 ===================== */
/* 视觉->MCU 最大载荷：0x23/0x26/0x33/0x43 的 dx,dy（4B） */
#define VISION_LINK_MAX_RX_DATA              (4U)
/* MCU->视觉 最大载荷：0x14 的 x/y/z（6B） */
#define VISION_LINK_MAX_TX_DATA              (6U)
/* 低频帧环形队列，必须是 2 的幂 */
#define VISION_LINK_RX_QUEUE_LEN             (8U)
/* 事件环形队列，必须是 2 的幂 */
#define VISION_LINK_EVENT_QUEUE_LEN          (16U)
/* 追踪最新值槽数量：B 物体 / B 桶 / C / D */
#define VISION_LINK_TRACK_SLOT_COUNT         (4U)

/* ===================== 2. 链路时序 ===================== */
/* 协议 §1：连续 500ms 收不到视觉任何帧判失联 */
#define VISION_LINK_LINK_TIMEOUT_MS          (500U)
/* 残帧复位：115200 下一帧 <1ms，100ms 仅作异常兜底 */
#define VISION_LINK_FRAME_TIMEOUT_MS         (100U)
/* USART3 发送超时：沿用工程现有驱动的 20ms */
#define VISION_LINK_UART_TX_TIMEOUT_MS       (20U)
/* 每拍最多发送帧数（ACK + 重发 + 位姿） */
#define VISION_LINK_TX_MAX_FRAMES_PER_UPDATE (4U)
/* 每拍最多 ACK 数：5B@115200 约 0.43ms */
#define VISION_LINK_ACK_MAX_PER_UPDATE       (2U)

/* ===================== 3. 请求-应答事务（协议 §6） ===================== */
/* 扫码：1s 间隔，总共发送 3 次 */
#define VISION_LINK_SCAN_RETRY_MS            (1000U)
#define VISION_LINK_SCAN_MAX_TX              (3U)
/* 扫码：最后一次发送后 1s 仍无响应判失败 */
#define VISION_LINK_SCAN_RESPONSE_TIMEOUT_MS (1000U)
/* 中位线：500ms 无回包重发，可反复请求 */
#define VISION_LINK_MIDLINE_RETRY_MS         (500U)
/* 顺序判定：3s 无回包按 1=中兜底，不停车 */
#define VISION_LINK_ORDER_TIMEOUT_MS         (3000U)
#define VISION_LINK_ORDER_FALLBACK_VALUE     (1U)

/* ===================== 4. 追踪 ===================== */
/* 追踪：500ms 无偏移判丢 */
#define VISION_LINK_TRACK_LOSS_MS            (500U)
/* 连续 5 次丢失 -> 上报后由业务回展开态 */
#define VISION_LINK_TRACK_LOSS_COUNT         (5U)
/* 协议 §5.3：建议 50Hz */
#define VISION_LINK_TRACK_DEFAULT_HZ         (50U)
#define VISION_LINK_TRACK_HZ_MAX             (200U)

/* ===================== 5. 位姿上报 ===================== */
/* 0x14 最多 50Hz，新值覆盖旧值 */
#define VISION_LINK_POSE_MIN_INTERVAL_MS     (20U)

/* ===================== 6. 协议命令号（v1.5.1） ===================== */
#define VISION_CMD_HEARTBEAT                 (0xFEU)
#define VISION_CMD_ACK                       (0xFFU)

/* A 点扫码 */
#define VISION_CMD_SCAN_CODE_REQ             (0x10U)
#define VISION_CMD_SCAN_CODE_OK              (0x11U)
#define VISION_CMD_ERROR                     (0x12U)
#define VISION_CMD_TARGET_LOST               (0x13U)
#define VISION_CMD_TCP_POSE                  (0x14U)

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
