/**
  ******************************************************************************
  * @file    app/vision_link.c
  * @brief   视觉链路：USART3 A5 帧收发、XOR、ACK、心跳失联、重试与追踪看门狗。
  *
  * 中断内只做字节状态机、XOR、大端转换与 volatile 更新；
  * ACK、命令与重发全部在任务上下文发送。
  * 本文件不调用任何底盘/机械臂接口，失联与追踪丢失只通过事件上报。
  ******************************************************************************
  */

#include "app/vision_link.h"
#include "app/vision_link_config.h"
#include "common/uart_rx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart3;

/* 帧解析状态 */
#define VF_HEAD     (0U)
#define VF_CMD      (1U)
#define VF_LEN      (2U)
#define VF_DATA     (3U)
#define VF_XOR      (4U)

/* pending 事务类型 */
#define VISION_PEND_NONE    (0U)
#define VISION_PEND_SCAN    (1U)
#define VISION_PEND_MIDLINE (2U)
#define VISION_PEND_ORDER   (3U)

/* ACK 位图字数：256 bit */
#define VISION_ACK_WORDS    (8U)

typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[VISION_LINK_MAX_RX_DATA];
} Vision_Msg_t;

typedef struct
{
    volatile int16_t  dx_mm;
    volatile int16_t  dy_mm;
    volatile uint32_t tick_ms;
    volatile uint32_t seq;
} Vision_TrackSlot_t;

typedef struct
{
    uint8_t  kind;
    uint8_t  zone;
    uint8_t  tx_cmd;
    uint8_t  tx_error_reported;
    uint16_t tx_count;
    uint32_t next_tx_ms;
    uint32_t deadline_ms;
} Vision_Pending_t;

/* ===================== 静态状态 ===================== */

/* ISR 与任务共享的解析状态；任务侧复位时进入短临界区 */
static volatile uint8_t  s_parser_state;
static volatile uint8_t  s_parser_synced;
static volatile uint8_t  s_parser_active;
static volatile uint32_t s_parser_last_ms;
static uint8_t           s_rx_cmd;
static uint8_t           s_rx_len;
static uint8_t           s_rx_idx;
static uint8_t           s_rx_xor;
static uint8_t           s_rx_buf[VISION_LINK_MAX_RX_DATA];

/* ISR -> 任务：低频帧 SPSC 环形队列 */
static Vision_Msg_t      s_rxq[VISION_LINK_RX_QUEUE_LEN];
static volatile uint8_t  s_rx_head;
static volatile uint8_t  s_rx_tail;

/* 追踪流最新值槽，ISR 覆盖写 */
static Vision_TrackSlot_t s_track[VISION_LINK_TRACK_SLOT_COUNT];

/* 心跳/链路时间戳 */
static volatile uint32_t s_last_rx_ms;
static volatile uint8_t  s_rx_seen;

/* 待回 ACK 位图：ISR 置位，任务临界区取位 */
static volatile uint32_t s_ack_mask[VISION_ACK_WORDS];

/* 以下状态只在任务上下文读写 */
static uint8_t  s_online;
static uint8_t  s_ever_online;
static Vision_Pending_t s_pending;
static uint8_t  s_track_active;
static uint32_t s_last_seen_seq;
static uint8_t  s_loss_count;
static uint8_t  s_loss_reported;
static uint8_t  s_loss5_reported;
static uint32_t s_loss_next_ms;
static uint8_t  s_update_tx_left;

/* 事件环，只在任务上下文生产/消费 */
static Vision_Event_t s_evq[VISION_LINK_EVENT_QUEUE_LEN];
static uint8_t s_ev_head;
static uint8_t s_ev_tail;

static volatile Vision_LinkStats_t s_stats;

/* ===================== 小工具 ===================== */

static uint16_t vision_get_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint8_t vision_zone_cmd(Vision_Zone_e zone, uint8_t base)
{
    switch (zone)
    {
    case VISION_ZONE_B:
        return base;
    case VISION_ZONE_C:
        return (uint8_t)(base + 0x10U);
    case VISION_ZONE_D:
        return (uint8_t)(base + 0x20U);
    default:
        return 0U;
    }
}

static uint8_t vision_track_start_cmd(uint8_t target)
{
    switch (target)
    {
    case VISION_TRACK_B_OBJECT:
        return VISION_CMD_B_TRACK_START;
    case VISION_TRACK_B_BUCKET:
        return VISION_CMD_B_BUCKET_START;
    case VISION_TRACK_C:
        return VISION_CMD_C_TRACK_START;
    case VISION_TRACK_D:
        return VISION_CMD_D_TRACK_START;
    default:
        return 0U;
    }
}

static uint8_t vision_track_stop_cmd(uint8_t target)
{
    switch (target)
    {
    case VISION_TRACK_B_OBJECT:
        return VISION_CMD_B_TRACK_STOP;
    case VISION_TRACK_B_BUCKET:
        return VISION_CMD_B_BUCKET_STOP;
    case VISION_TRACK_C:
        return VISION_CMD_C_TRACK_STOP;
    case VISION_TRACK_D:
        return VISION_CMD_D_TRACK_STOP;
    default:
        return 0U;
    }
}

static uint8_t vision_track_index(uint8_t target)
{
    switch (target)
    {
    case VISION_TRACK_B_OBJECT:
        return 0U;
    case VISION_TRACK_B_BUCKET:
        return 1U;
    case VISION_TRACK_C:
        return 2U;
    case VISION_TRACK_D:
        return 3U;
    default:
        return 0xFFU;
    }
}

static uint8_t vision_track_flow_index(uint8_t cmd, uint8_t *idx)
{
    switch (cmd)
    {
    case VISION_CMD_B_TRACK_OFFSET:
        *idx = 0U;
        return 1U;
    case VISION_CMD_B_BUCKET_OFFSET:
        *idx = 1U;
        return 1U;
    case VISION_CMD_C_TRACK_OFFSET:
        *idx = 2U;
        return 1U;
    case VISION_CMD_D_TRACK_OFFSET:
        *idx = 3U;
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t vision_track_zone(uint8_t target)
{
    switch (target)
    {
    case VISION_TRACK_B_OBJECT:
    case VISION_TRACK_B_BUCKET:
        return VISION_ZONE_B;
    case VISION_TRACK_C:
        return VISION_ZONE_C;
    case VISION_TRACK_D:
        return VISION_ZONE_D;
    default:
        return VISION_ZONE_NONE;
    }
}

static uint8_t vision_response_zone(uint8_t cmd)
{
    switch (cmd)
    {
    case VISION_CMD_B_MIDLINE_RSP:
    case VISION_CMD_B_ORDER_RSP:
        return VISION_ZONE_B;
    case VISION_CMD_C_MIDLINE_RSP:
    case VISION_CMD_C_ORDER_RSP:
        return VISION_ZONE_C;
    case VISION_CMD_D_MIDLINE_RSP:
    case VISION_CMD_D_ORDER_RSP:
        return VISION_ZONE_D;
    default:
        return VISION_ZONE_NONE;
    }
}

static uint8_t vision_cmd_known(uint8_t cmd)
{
    switch (cmd)
    {
    case VISION_CMD_SCAN_CODE_OK:
    case VISION_CMD_ERROR:
    case VISION_CMD_TARGET_LOST:
    case VISION_CMD_C_TARGET_DOWN:
    case VISION_CMD_B_MIDLINE_RSP:
    case VISION_CMD_C_MIDLINE_RSP:
    case VISION_CMD_D_MIDLINE_RSP:
    case VISION_CMD_B_ORDER_RSP:
    case VISION_CMD_C_ORDER_RSP:
    case VISION_CMD_D_ORDER_RSP:
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t vision_cmd_payload_ok(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    switch (cmd)
    {
    case VISION_CMD_HEARTBEAT:
        /* 心跳（v1.5.1 §4）是上位机单向 5Hz 空载荷帧，收到不回 ACK。
           以前这里没有分支，心跳落到 default 被当成非法载荷，err_payload 每拍
           涨一次，现场排查时会误以为协议出错。 */
        return (len == 0U) ? 1U : 0U;

    case VISION_CMD_SCAN_CODE_OK:
    case VISION_CMD_TARGET_LOST:
    case VISION_CMD_C_TARGET_DOWN:
        return (len == 0U) ? 1U : 0U;

    case VISION_CMD_ERROR:
        return ((len == 1U) && (data[0] >= 1U) && (data[0] <= 6U)) ? 1U : 0U;

    case VISION_CMD_B_MIDLINE_RSP:
    case VISION_CMD_C_MIDLINE_RSP:
    case VISION_CMD_D_MIDLINE_RSP:
        return ((len == 1U) && (data[0] <= 3U)) ? 1U : 0U;

    case VISION_CMD_B_ORDER_RSP:
    case VISION_CMD_C_ORDER_RSP:
    case VISION_CMD_D_ORDER_RSP:
        return ((len == 1U) && (data[0] <= 2U)) ? 1U : 0U;

    default:
        return 0U;
    }
}

static uint8_t vision_ack_exempt(uint8_t cmd)
{
    switch (cmd)
    {
    case VISION_CMD_B_TRACK_OFFSET:
    case VISION_CMD_B_BUCKET_OFFSET:
    case VISION_CMD_C_TRACK_OFFSET:
    case VISION_CMD_D_TRACK_OFFSET:
    case VISION_CMD_HEARTBEAT:
    case VISION_CMD_ACK:
        return 1U;
    default:
        return 0U;
    }
}

static void vision_ack_set(uint8_t cmd)
{
    s_ack_mask[cmd >> 5] |= (uint32_t)(1UL << (cmd & 31U));
}

/* ===================== ISR -> 任务队列 ===================== */

static uint8_t vision_rxq_push(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    uint8_t h = s_rx_head;
    uint8_t n = (uint8_t)((h + 1U) & (VISION_LINK_RX_QUEUE_LEN - 1U));
    uint8_t i;

    if (n == s_rx_tail)
    {
        return 0U;
    }

    s_rxq[h].cmd = cmd;
    s_rxq[h].len = len;
    for (i = 0U; i < len; i++)
    {
        s_rxq[h].data[i] = data[i];
    }

    __DMB();                /* 槽位写完再发布 head */
    s_rx_head = n;
    return 1U;
}

static uint8_t vision_rxq_pop(Vision_Msg_t *out)
{
    uint8_t t = s_rx_tail;

    if (t == s_rx_head)
    {
        return 0U;
    }

    __DMB();                /* 先看到 head，再读槽位 */
    out->cmd = s_rxq[t].cmd;
    out->len = s_rxq[t].len;
    out->data[0] = s_rxq[t].data[0];
    out->data[1] = s_rxq[t].data[1];
    out->data[2] = s_rxq[t].data[2];
    out->data[3] = s_rxq[t].data[3];
    __DMB();
    s_rx_tail = (uint8_t)((t + 1U) & (VISION_LINK_RX_QUEUE_LEN - 1U));
    return 1U;
}

/* ===================== ISR：帧解析与分派 ===================== */

static void vision_frame_dispatch(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    uint8_t idx;

    s_stats.rx_frames++;
    s_last_rx_ms = HAL_GetTick();
    s_rx_seen = 1U;

    /* 追踪流：只更新最新值槽，不排队、不回 ACK */
    if (vision_track_flow_index(cmd, &idx) != 0U)
    {
        if ((len == 4U) && (idx < VISION_LINK_TRACK_SLOT_COUNT))
        {
            s_track[idx].dx_mm = (int16_t)vision_get_be16(&data[0]);
            s_track[idx].dy_mm = (int16_t)vision_get_be16(&data[2]);
            s_track[idx].tick_ms = s_last_rx_ms;
            s_track[idx].seq++;
        }
        else
        {
            s_stats.err_payload++;
        }
        return;
    }

    if (cmd == VISION_CMD_HEARTBEAT)
    {
        if (len != 0U)
        {
            s_stats.err_payload++;
        }
        return;
    }

    if (cmd == VISION_CMD_ACK)
    {
        s_stats.err_unexpected++;
        return;
    }

    if (vision_cmd_known(cmd) == 0U)
    {
        s_stats.err_unknown++;
        return;
    }

    if (vision_cmd_payload_ok(cmd, len, data) == 0U)
    {
        s_stats.err_payload++;
        return;
    }

    /* 未入队则不 ACK，避免“确认了但事件已丢” */
    if (vision_rxq_push(cmd, len, data) == 0U)
    {
        s_stats.rx_overflow++;
        return;
    }

    if (vision_ack_exempt(cmd) == 0U)
    {
        vision_ack_set(cmd);
    }
}

static void vision_rx_byte_isr(uint8_t byte)
{
    s_parser_last_ms = HAL_GetTick();

    switch (s_parser_state)
    {
    case VF_HEAD:
        if (byte == 0xA5U)
        {
            s_parser_state = VF_CMD;
            s_parser_synced = 1U;
        }
        else if (s_parser_synced != 0U)
        {
            s_stats.err_noise++;    /* 一次失步只计一次 */
            s_parser_synced = 0U;
        }
        break;

    case VF_CMD:
        s_rx_cmd = byte;
        s_rx_xor = byte;
        s_parser_state = VF_LEN;
        break;

    case VF_LEN:
        s_rx_len = byte;
        if (s_rx_len > VISION_LINK_MAX_RX_DATA)
        {
            s_stats.err_len++;
            s_parser_synced = 0U;
            s_parser_state = VF_HEAD;
            break;
        }
        s_rx_xor ^= byte;
        s_rx_idx = 0U;
        s_parser_state = (s_rx_len == 0U) ? VF_XOR : VF_DATA;
        break;

    case VF_DATA:
        s_rx_buf[s_rx_idx] = byte;
        s_rx_xor ^= byte;
        s_rx_idx++;
        if (s_rx_idx >= s_rx_len)
        {
            s_parser_state = VF_XOR;
        }
        break;

    case VF_XOR:
        if (byte == s_rx_xor)
        {
            vision_frame_dispatch(s_rx_cmd, s_rx_len, s_rx_buf);
            s_parser_synced = 1U;
        }
        else
        {
            s_stats.err_xor++;
            s_parser_synced = 0U;
        }
        s_parser_state = VF_HEAD;
        break;

    default:
        s_parser_state = VF_HEAD;
        break;
    }

    s_parser_active = (s_parser_state != VF_HEAD) ? 1U : 0U;
}

/* ===================== 发送 ===================== */

/* 审计 H5：发送改 IT + 单生产者/单消费者环形缓冲。
 * 生产者只有 MainTask02（命令、重发、ACK 全在该任务上下文），消费者只有
 * USART3 的 TxCplt 中断；head 只由任务写、tail 只由中断写，两者都是单字写入，
 * 因此不需要临界区，也就不会出现「任务持锁等 UART」的阻塞。
 * 帧以整帧拷入缓冲，帧与帧之间不会交叉；顺序严格 FIFO，与原先逐帧阻塞发送一致。 */
static uint8_t  s_tx_ring[VISION_LINK_TX_RING_SIZE];
static volatile uint16_t s_tx_head;        /* 任务侧写入位置 */
static volatile uint16_t s_tx_tail;        /* 中断侧已发/在发位置 */
static volatile uint16_t s_tx_chunk;       /* HAL 当前正在搬的字节数 */
static volatile uint8_t  s_tx_busy;        /* 1 = HAL 正在搬 */
static volatile uint32_t s_tx_progress_ms; /* 最近一次进展时刻，供停滞兜底 */

/* 把缓冲里待发的一段交给 HAL；只能在 s_tx_busy == 0 时调用（任务或中断均可） */
static void vision_tx_kick(void)
{
    uint16_t tail = s_tx_tail;
    uint16_t head = s_tx_head;
    uint16_t chunk;

    if ((s_tx_busy != 0U) || (tail == head))
    {
        return;
    }

    /* 只发到缓冲末尾，回绕的剩余部分由 TxCplt 续发 */
    chunk = (head > tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(VISION_LINK_TX_RING_SIZE - tail);

    s_tx_chunk = chunk;
    s_tx_busy = 1U;
    s_tx_progress_ms = HAL_GetTick();

    if (HAL_UART_Transmit_IT(&huart3, &s_tx_ring[tail], chunk) != HAL_OK)
    {
        s_tx_busy = 0U;
        s_tx_chunk = 0U;
        s_stats.tx_errors++;
    }
}

/* 入队一整帧；缓冲放不下就不入队（返回失败，帧不会被截断） */
static int vision_tx_enqueue(const uint8_t *buf, uint16_t n)
{
    uint16_t used;
    uint16_t i;
    uint16_t mask = (uint16_t)(VISION_LINK_TX_RING_SIZE - 1U);

    used = (uint16_t)((s_tx_head - s_tx_tail) & mask);
    if ((uint16_t)(mask - used) < n)
    {
        s_stats.tx_errors++;
        return VISION_LINK_ERR_UART;
    }

    for (i = 0U; i < n; i++)
    {
        s_tx_ring[(uint16_t)((s_tx_head + i) & mask)] = buf[i];
    }
    __DMB();                 /* 先写完字节，再发布 head */
    s_tx_head = (uint16_t)((s_tx_head + n) & mask);

    vision_tx_kick();

    return VISION_LINK_OK;
}

/* TxCplt：推进 tail 并续发下一段。只由 USART3 中断调用 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t mask = (uint16_t)(VISION_LINK_TX_RING_SIZE - 1U);

    if ((huart == NULL) || (huart->Instance != USART3) || (s_tx_busy == 0U))
    {
        return;
    }

    s_tx_tail = (uint16_t)((s_tx_tail + s_tx_chunk) & mask);
    s_tx_chunk = 0U;
    s_tx_busy = 0U;
    s_tx_progress_ms = HAL_GetTick();

    vision_tx_kick();
}

/* 停滞兜底：IT 因错误被 HAL 终结时不会再回调 TxCplt，这里强制复位，
 * 丢掉本轮未发完的半帧（上位机按 XOR 校验丢帧，下一帧仍从 0xA5 重新同步） */
static void vision_tx_stall_service(uint32_t now)
{
    if ((s_tx_busy != 0U)
        && ((uint32_t)(now - s_tx_progress_ms) > VISION_LINK_TX_STALL_MS))
    {
        (void)HAL_UART_AbortTransmit(&huart3);
        s_tx_tail = (uint16_t)((s_tx_tail + s_tx_chunk)
                               & (uint16_t)(VISION_LINK_TX_RING_SIZE - 1U));
        s_tx_chunk = 0U;
        s_tx_busy = 0U;
        s_stats.tx_errors++;
    }

    /* 缓冲里有数据但没人搬（例如上一段刚结束、入队发生在 kick 之后）：补一脚 */
    if (s_tx_busy == 0U)
    {
        vision_tx_kick();
    }
}

static int vision_send_raw(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[VISION_LINK_MAX_TX_DATA + 4U];
    uint8_t x;
    uint8_t n;
    uint8_t i;

    if (len > VISION_LINK_MAX_TX_DATA)
    {
        return VISION_LINK_ERR_ARG;
    }

    buf[0] = 0xA5U;
    buf[1] = cmd;
    buf[2] = len;
    x = (uint8_t)(cmd ^ len);

    for (i = 0U; i < len; i++)
    {
        buf[3U + i] = data[i];
        x ^= data[i];
    }
    buf[3U + len] = x;
    n = (uint8_t)(len + 4U);

    s_stats.tx_frames++;

    /* 帧格式（A5/CMD/LEN/DATA/XOR、命令号、载荷长度）完全不变，只是不再等发送完毕 */
    return vision_tx_enqueue(buf, n);
}

/* Update 内部发送：受每拍帧数预算限制；失败后本拍不再发送 */
static int vision_update_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    int ret;

    if (s_update_tx_left == 0U)
    {
        return VISION_LINK_ERR_NO_BUDGET;
    }
    s_update_tx_left--;

    ret = vision_send_raw(cmd, data, len);
    if (ret != VISION_LINK_OK)
    {
        s_update_tx_left = 0U;
    }

    return ret;
}

/* ===================== 事件 ===================== */

static uint8_t vision_emit(uint8_t type, uint8_t zone, uint8_t cmd,
                           uint8_t value, uint8_t reason, uint8_t flags)
{
    uint8_t n = (uint8_t)((s_ev_head + 1U) & (VISION_LINK_EVENT_QUEUE_LEN - 1U));

    if (n == s_ev_tail)
    {
        /* 队列满：丢弃最旧事件，保证最新关键事件能入队 */
        s_ev_tail = (uint8_t)((s_ev_tail + 1U) & (VISION_LINK_EVENT_QUEUE_LEN - 1U));
        s_stats.event_overflow++;
    }

    s_evq[s_ev_head].tick_ms = HAL_GetTick();
    s_evq[s_ev_head].type = type;
    s_evq[s_ev_head].zone = zone;
    s_evq[s_ev_head].cmd = cmd;
    s_evq[s_ev_head].value = value;
    s_evq[s_ev_head].reason = reason;
    s_evq[s_ev_head].flags = flags;
    s_evq[s_ev_head].reserved[0] = 0U;
    s_evq[s_ev_head].reserved[1] = 0U;

    s_ev_head = n;
    return 1U;
}

/* ===================== 任务侧服务 ===================== */

static uint8_t vision_online_now(uint32_t now)
{
    uint8_t seen;
    uint32_t last;

    taskENTER_CRITICAL();
    seen = s_rx_seen;
    last = s_last_rx_ms;
    taskEXIT_CRITICAL();

    if (seen == 0U)
    {
        return 0U;
    }

    return ((uint32_t)(now - last) < VISION_LINK_LINK_TIMEOUT_MS) ? 1U : 0U;
}

static void vision_pending_clear(void)
{
    s_pending.kind = VISION_PEND_NONE;
    s_pending.zone = VISION_ZONE_NONE;
    s_pending.tx_cmd = 0U;
    s_pending.tx_error_reported = 0U;
    s_pending.tx_count = 0U;
    s_pending.next_tx_ms = 0U;
    s_pending.deadline_ms = 0U;
}

static void vision_track_reset_loss(void)
{
    s_last_seen_seq = 0U;
    s_loss_count = 0U;
    s_loss_reported = 0U;
    s_loss5_reported = 0U;
    s_loss_next_ms = 0U;
}

static void vision_parser_timeout_service(uint32_t now)
{
    uint8_t active;
    uint32_t last;

    taskENTER_CRITICAL();
    active = s_parser_active;
    last = s_parser_last_ms;
    taskEXIT_CRITICAL();

    if ((active != 0U) && ((uint32_t)(now - last) > VISION_LINK_FRAME_TIMEOUT_MS))
    {
        taskENTER_CRITICAL();
        s_parser_state = VF_HEAD;
        s_parser_synced = 0U;
        s_parser_active = 0U;
        taskEXIT_CRITICAL();
        s_stats.err_frame_timeout++;
    }
}

/* 失联：作废事务、清追踪与待发位姿、清队列与 ACK，不向视觉发任何帧 */
static void vision_abort_all(void)
{
    uint8_t i;

    if (s_pending.kind != VISION_PEND_NONE)
    {
        vision_emit(VISION_EV_TXN_ABORTED, s_pending.zone, s_pending.tx_cmd,
                    0U, VISION_REASON_LINK_LOST, 0U);
        vision_pending_clear();
    }

    if (s_track_active != VISION_TRACK_NONE)
    {
        s_track_active = VISION_TRACK_NONE;
        vision_track_reset_loss();
    }

    taskENTER_CRITICAL();
    for (i = 0U; i < VISION_ACK_WORDS; i++)
    {
        s_ack_mask[i] = 0U;
    }
    s_rx_tail = s_rx_head;
    s_parser_state = VF_HEAD;
    s_parser_synced = 0U;
    s_parser_active = 0U;
    taskEXIT_CRITICAL();
}

static void vision_link_state_service(uint32_t now)
{
    if (vision_online_now(now) != 0U)
    {
        if (s_online == 0U)
        {
            s_online = 1U;
            if (s_ever_online != 0U)
            {
                vision_emit(VISION_EV_LINK_RECOVERED, VISION_ZONE_NONE, 0U,
                            0U, VISION_REASON_NONE, 0U);
            }
            s_ever_online = 1U;
        }
    }
    else if (s_online != 0U)
    {
        s_online = 0U;
        vision_emit(VISION_EV_LINK_LOST, VISION_ZONE_NONE, 0U,
                    0U, VISION_REASON_NONE, 0U);
        vision_abort_all();
    }
}

static void vision_rx_drain(void)
{
    Vision_Msg_t msg;
    uint8_t zone;

    while (vision_rxq_pop(&msg) != 0U)
    {
        switch (msg.cmd)
        {
        case VISION_CMD_SCAN_CODE_OK:
            if (s_pending.kind == VISION_PEND_SCAN)
            {
                vision_emit(VISION_EV_SCAN_OK, VISION_ZONE_NONE, msg.cmd,
                            0U, VISION_REASON_NONE, 0U);
                vision_pending_clear();
            }
            else
            {
                s_stats.late_responses++;
            }
            break;

        case VISION_CMD_ERROR:
            if (s_pending.kind == VISION_PEND_SCAN)
            {
                vision_emit(VISION_EV_SCAN_ERROR, VISION_ZONE_NONE, s_pending.tx_cmd,
                            msg.data[0], VISION_REASON_NONE, 0U);
                vision_pending_clear();
            }
            else if (s_pending.kind == VISION_PEND_MIDLINE)
            {
                vision_emit(VISION_EV_VISION_ERROR, s_pending.zone, s_pending.tx_cmd,
                            msg.data[0], VISION_REASON_NONE, 0U);
                vision_pending_clear();
            }
            else if (s_pending.kind == VISION_PEND_ORDER)
            {
                /* 顺序事务收到错误：按中位兜底，flags bit0=1 */
                vision_emit(VISION_EV_ORDER, s_pending.zone, s_pending.tx_cmd,
                            VISION_LINK_ORDER_FALLBACK_VALUE, VISION_REASON_NONE, 0x01U);
                vision_pending_clear();
            }
            else
            {
                vision_emit(VISION_EV_VISION_ERROR, VISION_ZONE_NONE, msg.cmd,
                            msg.data[0], VISION_REASON_NONE, 0U);
            }

            /* 0x05 追踪 60s 超时：本地追踪同步失效，避免再报 5 次窗口 */
            if (msg.data[0] == 5U)
            {
                s_track_active = VISION_TRACK_NONE;
                vision_track_reset_loss();
            }
            break;

        case VISION_CMD_TARGET_LOST:
            if (s_track_active != VISION_TRACK_NONE)
            {
                if (s_loss_reported == 0U)
                {
                    vision_emit(VISION_EV_TARGET_LOST, vision_track_zone(s_track_active),
                                msg.cmd, 0U, VISION_REASON_NONE, 0U);
                    s_loss_reported = 1U;
                }
            }
            else
            {
                s_stats.err_unexpected++;
            }
            break;

        case VISION_CMD_B_MIDLINE_RSP:
        case VISION_CMD_C_MIDLINE_RSP:
        case VISION_CMD_D_MIDLINE_RSP:
            zone = vision_response_zone(msg.cmd);
            if ((s_pending.kind == VISION_PEND_MIDLINE) && (s_pending.zone == zone))
            {
                vision_emit(VISION_EV_MIDLINE, zone, msg.cmd,
                            msg.data[0], VISION_REASON_NONE, 0U);
                vision_pending_clear();
            }
            else
            {
                s_stats.late_responses++;
            }
            break;

        case VISION_CMD_B_ORDER_RSP:
        case VISION_CMD_C_ORDER_RSP:
        case VISION_CMD_D_ORDER_RSP:
            zone = vision_response_zone(msg.cmd);
            if ((s_pending.kind == VISION_PEND_ORDER) && (s_pending.zone == zone))
            {
                vision_emit(VISION_EV_ORDER, zone, msg.cmd,
                            msg.data[0], VISION_REASON_NONE, 0U);
                vision_pending_clear();
            }
            else
            {
                s_stats.late_responses++;
            }
            break;

        case VISION_CMD_C_TARGET_DOWN:
            vision_emit(VISION_EV_C_TARGET_DOWN, VISION_ZONE_C, msg.cmd,
                        0U, VISION_REASON_NONE, 0U);
            break;

        default:
            break;
        }
    }
}

static void vision_pending_service(uint32_t now)
{
    int ret;

    if (s_pending.kind == VISION_PEND_NONE)
    {
        return;
    }

    /* 失联路径已统一作废，这里不重复处理 */
    if (vision_online_now(now) == 0U)
    {
        return;
    }

    switch (s_pending.kind)
    {
    case VISION_PEND_SCAN:
        if ((s_pending.tx_count < VISION_LINK_SCAN_MAX_TX)
            && ((int32_t)(now - s_pending.next_tx_ms) >= 0))
        {
            ret = vision_update_send(s_pending.tx_cmd, NULL, 0U);
            if (ret == VISION_LINK_ERR_NO_BUDGET)
            {
                break;
            }
            if ((ret == VISION_LINK_ERR_UART) && (s_pending.tx_error_reported == 0U))
            {
                s_pending.tx_error_reported = 1U;
                vision_emit(VISION_EV_TX_ERROR, s_pending.zone, s_pending.tx_cmd,
                            0U, VISION_REASON_UART, 0U);
            }
            s_pending.tx_count++;
            s_pending.next_tx_ms = now + VISION_LINK_SCAN_RETRY_MS;
            s_pending.deadline_ms = now + VISION_LINK_SCAN_RESPONSE_TIMEOUT_MS;
            s_stats.retries++;
        }
        else if ((s_pending.tx_count >= VISION_LINK_SCAN_MAX_TX)
                 && ((int32_t)(now - s_pending.deadline_ms) >= 0))
        {
            vision_emit(VISION_EV_SCAN_TIMEOUT, VISION_ZONE_NONE, s_pending.tx_cmd,
                        0U, VISION_REASON_TIMEOUT, 0U);
            vision_pending_clear();
        }
        break;

    case VISION_PEND_MIDLINE:
        /* 审计 H6：pending 必须有生命周期上限，否则一旦没人 CancelMidline，
         * 整个视觉链路会被 RequestOrder/RequestScanCode/TrackStart 的 ERR_BUSY 永久堵死。
         * 超时只上报事件并清 pending，不重发、不停车，控制权交回业务层。 */
        if ((int32_t)(now - s_pending.deadline_ms) >= 0)
        {
            vision_emit(VISION_EV_VISION_ERROR, s_pending.zone, s_pending.tx_cmd,
                        0U, VISION_REASON_TIMEOUT, 0U);
            vision_pending_clear();
            break;
        }

        if ((int32_t)(now - s_pending.next_tx_ms) >= 0)
        {
            ret = vision_update_send(s_pending.tx_cmd, NULL, 0U);
            if (ret == VISION_LINK_ERR_NO_BUDGET)
            {
                break;
            }
            if ((ret == VISION_LINK_ERR_UART) && (s_pending.tx_error_reported == 0U))
            {
                s_pending.tx_error_reported = 1U;
                vision_emit(VISION_EV_TX_ERROR, s_pending.zone, s_pending.tx_cmd,
                            0U, VISION_REASON_UART, 0U);
            }
            s_pending.tx_count++;
            s_pending.next_tx_ms = now + VISION_LINK_MIDLINE_RETRY_MS;
            s_stats.retries++;
        }
        break;

    case VISION_PEND_ORDER:
        if ((int32_t)(now - s_pending.deadline_ms) >= 0)
        {
            vision_emit(VISION_EV_ORDER, s_pending.zone, s_pending.tx_cmd,
                        VISION_LINK_ORDER_FALLBACK_VALUE, VISION_REASON_TIMEOUT, 0x01U);
            vision_pending_clear();
        }
        break;

    default:
        vision_pending_clear();
        break;
    }
}

static void vision_track_service(uint32_t now)
{
    uint8_t idx;
    uint32_t seq;

    if (s_track_active == VISION_TRACK_NONE)
    {
        return;
    }

    idx = vision_track_index(s_track_active);
    if (idx >= VISION_LINK_TRACK_SLOT_COUNT)
    {
        s_track_active = VISION_TRACK_NONE;
        return;
    }

    taskENTER_CRITICAL();
    seq = s_track[idx].seq;
    taskEXIT_CRITICAL();

    /* 新样本到达：清空丢失窗口，重新开始计时 */
    if (seq != s_last_seen_seq)
    {
        s_last_seen_seq = seq;
        s_loss_count = 0U;
        s_loss_reported = 0U;
        s_loss5_reported = 0U;
        s_loss_next_ms = now + VISION_LINK_TRACK_LOSS_MS;
        return;
    }

    if ((int32_t)(now - s_loss_next_ms) < 0)
    {
        return;
    }

    while (((int32_t)(now - s_loss_next_ms) >= 0)
           && (s_loss_count < VISION_LINK_TRACK_LOSS_COUNT))
    {
        s_loss_count++;
        s_loss_next_ms += VISION_LINK_TRACK_LOSS_MS;
    }

    if ((s_loss_count > 0U) && (s_loss_reported == 0U))
    {
        vision_emit(VISION_EV_TARGET_LOST, vision_track_zone(s_track_active),
                    0U, 0U, VISION_REASON_TIMEOUT, 0U);
        s_loss_reported = 1U;
    }

    if ((s_loss_count >= VISION_LINK_TRACK_LOSS_COUNT) && (s_loss5_reported == 0U))
    {
        vision_emit(VISION_EV_TRACK_LOST_5X, vision_track_zone(s_track_active),
                    0U, VISION_LINK_TRACK_LOSS_COUNT, VISION_REASON_TIMEOUT, 0U);
        s_loss5_reported = 1U;
        /* 清 active，允许业务对同一目标重新 TrackStart */
        s_track_active = VISION_TRACK_NONE;
    }
}

static void vision_ack_flush(void)
{
    uint8_t i;
    uint8_t sent = 0U;

    for (i = 0U; (i < VISION_ACK_WORDS) && (sent < VISION_LINK_ACK_MAX_PER_UPDATE); i++)
    {
        uint8_t cmd = 0U;
        uint8_t found = 0U;

        taskENTER_CRITICAL();
        if (s_ack_mask[i] != 0U)
        {
            uint32_t bits = s_ack_mask[i];
            uint8_t b = 0U;

            while (((bits >> b) & 1UL) == 0UL)
            {
                b++;
            }
            cmd = (uint8_t)((i << 5) + b);
            s_ack_mask[i] &= ~(uint32_t)(1UL << b);
            found = 1U;
        }
        taskEXIT_CRITICAL();

        if (found == 0U)
        {
            continue;
        }

        if (vision_update_send(VISION_CMD_ACK, &cmd, 1U) != VISION_LINK_OK)
        {
            return;
        }
        sent++;
    }
}

/* ===================== 对外接口 ===================== */

static void vision_stats_clear(void)
{
    s_stats.rx_frames = 0U;
    s_stats.tx_frames = 0U;
    s_stats.rx_overflow = 0U;
    s_stats.event_overflow = 0U;
    s_stats.err_noise = 0U;
    s_stats.err_len = 0U;
    s_stats.err_xor = 0U;
    s_stats.err_payload = 0U;
    s_stats.err_unknown = 0U;
    s_stats.err_frame_timeout = 0U;
    s_stats.late_responses = 0U;
    s_stats.err_unexpected = 0U;
    s_stats.retries = 0U;
    s_stats.tx_errors = 0U;
}

void Vision_Link_Init(void)
{
    uint8_t i;

    s_parser_state = VF_HEAD;
    s_parser_synced = 0U;
    s_parser_active = 0U;
    s_parser_last_ms = 0U;
    s_rx_cmd = 0U;
    s_rx_len = 0U;
    s_rx_idx = 0U;
    s_rx_xor = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;

    /* 审计 H5：发送缓冲复位 */
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_chunk = 0U;
    s_tx_busy = 0U;
    s_tx_progress_ms = 0U;

    for (i = 0U; i < VISION_LINK_TRACK_SLOT_COUNT; i++)
    {
        s_track[i].dx_mm = 0;
        s_track[i].dy_mm = 0;
        s_track[i].tick_ms = 0U;
        s_track[i].seq = 0U;
    }

    for (i = 0U; i < VISION_ACK_WORDS; i++)
    {
        s_ack_mask[i] = 0U;
    }

    s_last_rx_ms = 0U;
    s_rx_seen = 0U;
    s_online = 0U;
    s_ever_online = 0U;

    vision_pending_clear();
    s_track_active = VISION_TRACK_NONE;
    vision_track_reset_loss();

    s_update_tx_left = 0U;

    s_ev_head = 0U;
    s_ev_tail = 0U;

    vision_stats_clear();

    Uart_Rx_Register(&huart3, vision_rx_byte_isr);
    Uart_Rx_Start(&huart3);
}

void Vision_Link_Update(void)
{
    uint32_t now = HAL_GetTick();

    s_update_tx_left = VISION_LINK_TX_MAX_FRAMES_PER_UPDATE;

    vision_tx_stall_service(now);
    vision_parser_timeout_service(now);
    vision_link_state_service(now);
    vision_rx_drain();
    vision_pending_service(now);
    vision_track_service(now);
    vision_ack_flush();
}

uint8_t Vision_Link_IsOnline(void)
{
    return vision_online_now(HAL_GetTick());
}

int Vision_Link_RequestScanCode(void)
{
    uint32_t now;
    int ret;

    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if (s_pending.kind != VISION_PEND_NONE)
    {
        return VISION_LINK_ERR_BUSY;
    }

    now = HAL_GetTick();
    ret = vision_send_raw(VISION_CMD_SCAN_CODE_REQ, NULL, 0U);
    if (ret != VISION_LINK_OK)
    {
        return ret;
    }

    s_pending.kind = VISION_PEND_SCAN;
    s_pending.zone = VISION_ZONE_NONE;
    s_pending.tx_cmd = VISION_CMD_SCAN_CODE_REQ;
    s_pending.tx_error_reported = 0U;
    s_pending.tx_count = 1U;
    s_pending.next_tx_ms = now + VISION_LINK_SCAN_RETRY_MS;
    s_pending.deadline_ms = now + VISION_LINK_SCAN_RESPONSE_TIMEOUT_MS;

    return VISION_LINK_OK;
}

int Vision_Link_StartScan(Vision_Zone_e zone)
{
    uint8_t cmd = vision_zone_cmd(zone, VISION_CMD_B_SCAN);

    if (cmd == 0U)
    {
        return VISION_LINK_ERR_ARG;
    }
    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if ((s_pending.kind != VISION_PEND_NONE) || (s_track_active != VISION_TRACK_NONE))
    {
        return VISION_LINK_ERR_BUSY;
    }

    return vision_send_raw(cmd, NULL, 0U);
}

int Vision_Link_RequestMidline(Vision_Zone_e zone)
{
    uint8_t cmd = vision_zone_cmd(zone, VISION_CMD_B_MIDLINE_REQ);
    uint32_t now;
    int ret;

    if (cmd == 0U)
    {
        return VISION_LINK_ERR_ARG;
    }
    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if (s_pending.kind != VISION_PEND_NONE)
    {
        return VISION_LINK_ERR_BUSY;
    }

    now = HAL_GetTick();
    ret = vision_send_raw(cmd, NULL, 0U);
    if (ret != VISION_LINK_OK)
    {
        return ret;
    }

    s_pending.kind = VISION_PEND_MIDLINE;
    s_pending.zone = (uint8_t)zone;
    s_pending.tx_cmd = cmd;
    s_pending.tx_error_reported = 0U;
    s_pending.tx_count = 1U;
    s_pending.next_tx_ms = now + VISION_LINK_MIDLINE_RETRY_MS;
    /* 审计 H6：绝对截止时刻（不随重发滑动），到点自动结束事务 */
    s_pending.deadline_ms = now + VISION_LINK_MIDLINE_TIMEOUT_MS;

    return VISION_LINK_OK;
}

uint8_t Vision_Link_CancelMidline(void)
{
    if (s_pending.kind != VISION_PEND_MIDLINE)
    {
        return 0U;
    }

    vision_pending_clear();
    return 1U;
}

uint8_t Vision_Link_CancelPending(void)
{
    if (s_pending.kind == VISION_PEND_NONE)
    {
        return 0U;
    }

    vision_pending_clear();
    return 1U;
}

int Vision_Link_RequestOrder(Vision_Zone_e zone)
{
    uint8_t cmd = vision_zone_cmd(zone, VISION_CMD_B_ORDER_REQ);
    uint32_t now;
    int ret;

    if (cmd == 0U)
    {
        return VISION_LINK_ERR_ARG;
    }
    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if (s_pending.kind != VISION_PEND_NONE)
    {
        return VISION_LINK_ERR_BUSY;
    }

    now = HAL_GetTick();
    ret = vision_send_raw(cmd, NULL, 0U);
    if (ret != VISION_LINK_OK)
    {
        return ret;
    }

    s_pending.kind = VISION_PEND_ORDER;
    s_pending.zone = (uint8_t)zone;
    s_pending.tx_cmd = cmd;
    s_pending.tx_error_reported = 0U;
    s_pending.tx_count = 1U;
    s_pending.next_tx_ms = 0U;
    s_pending.deadline_ms = now + VISION_LINK_ORDER_TIMEOUT_MS;

    return VISION_LINK_OK;
}

int Vision_Link_TrackStart(Vision_TrackTarget_e target, uint16_t hz)
{
    uint8_t idx;
    uint8_t cmd;
    uint8_t data[2];
    uint32_t now;
    int ret;

    idx = vision_track_index((uint8_t)target);
    if (idx >= VISION_LINK_TRACK_SLOT_COUNT)
    {
        return VISION_LINK_ERR_ARG;
    }

    if (hz == 0U)
    {
        hz = VISION_LINK_TRACK_DEFAULT_HZ;
    }
    if (hz > VISION_LINK_TRACK_HZ_MAX)
    {
        return VISION_LINK_ERR_ARG;
    }

    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if (s_pending.kind != VISION_PEND_NONE)
    {
        return VISION_LINK_ERR_BUSY;
    }
    if ((s_track_active != VISION_TRACK_NONE) && (s_track_active != (uint8_t)target))
    {
        return VISION_LINK_ERR_BUSY;
    }
    if (s_track_active == (uint8_t)target)
    {
        return VISION_LINK_OK;
    }

    cmd = vision_track_start_cmd((uint8_t)target);
    data[0] = (uint8_t)(hz >> 8);
    data[1] = (uint8_t)hz;

    ret = vision_send_raw(cmd, data, 2U);
    if (ret != VISION_LINK_OK)
    {
        return ret;
    }

    now = HAL_GetTick();
    s_track_active = (uint8_t)target;

    taskENTER_CRITICAL();
    s_track[idx].dx_mm = 0;
    s_track[idx].dy_mm = 0;
    s_track[idx].tick_ms = 0U;
    s_track[idx].seq = 0U;
    taskEXIT_CRITICAL();

    vision_track_reset_loss();
    s_loss_next_ms = now + VISION_LINK_TRACK_LOSS_MS;

    return VISION_LINK_OK;
}

int Vision_Link_TrackStop(Vision_TrackTarget_e target)
{
    uint8_t cmd;
    int ret;

    cmd = vision_track_stop_cmd((uint8_t)target);
    if (cmd == 0U)
    {
        return VISION_LINK_ERR_ARG;
    }
    if (Vision_Link_IsOnline() == 0U)
    {
        return VISION_LINK_ERR_OFFLINE;
    }
    if ((s_track_active != VISION_TRACK_NONE) && (s_track_active != (uint8_t)target))
    {
        return VISION_LINK_ERR_STATE;
    }

    ret = vision_send_raw(cmd, NULL, 0U);
    if (ret != VISION_LINK_OK)
    {
        return ret;
    }

    if (s_track_active == (uint8_t)target)
    {
        s_track_active = VISION_TRACK_NONE;
        vision_track_reset_loss();
    }

    return VISION_LINK_OK;
}

uint8_t Vision_Link_GetTrack(Vision_TrackTarget_e target, Vision_TrackSample_t *out)
{
    uint8_t idx;
    int16_t dx;
    int16_t dy;
    uint32_t tick;
    uint32_t seq;
    uint32_t now;

    if ((out == NULL) || (s_track_active != (uint8_t)target))
    {
        return 0U;
    }

    idx = vision_track_index((uint8_t)target);
    if (idx >= VISION_LINK_TRACK_SLOT_COUNT)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    dx = s_track[idx].dx_mm;
    dy = s_track[idx].dy_mm;
    tick = s_track[idx].tick_ms;
    seq = s_track[idx].seq;
    taskEXIT_CRITICAL();

    if ((seq == 0U) || (tick == 0U))
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - tick) >= VISION_LINK_TRACK_LOSS_MS)
    {
        return 0U;
    }

    out->dx_mm = dx;
    out->dy_mm = dy;
    out->tick_ms = tick;
    out->seq = seq;
    out->age_ms = (uint32_t)(now - tick);
    out->target = (uint8_t)target;
    out->valid = 1U;

    return 1U;
}

uint8_t Vision_Link_TakeEvent(Vision_Event_t *ev)
{
    if ((ev == NULL) || (s_ev_tail == s_ev_head))
    {
        return 0U;
    }

    *ev = s_evq[s_ev_tail];
    s_ev_tail = (uint8_t)((s_ev_tail + 1U) & (VISION_LINK_EVENT_QUEUE_LEN - 1U));

    return 1U;
}

void Vision_Link_GetStats(Vision_LinkStats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    out->rx_frames = s_stats.rx_frames;
    out->tx_frames = s_stats.tx_frames;
    out->rx_overflow = s_stats.rx_overflow;
    out->event_overflow = s_stats.event_overflow;
    out->err_noise = s_stats.err_noise;
    out->err_len = s_stats.err_len;
    out->err_xor = s_stats.err_xor;
    out->err_payload = s_stats.err_payload;
    out->err_unknown = s_stats.err_unknown;
    out->err_frame_timeout = s_stats.err_frame_timeout;
    out->late_responses = s_stats.late_responses;
    out->err_unexpected = s_stats.err_unexpected;
    out->retries = s_stats.retries;
    out->tx_errors = s_stats.tx_errors;
    taskEXIT_CRITICAL();
}
