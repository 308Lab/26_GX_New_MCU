/**
  ******************************************************************************
  * @file    app/vision_link.h
  * @brief   视觉链路：A5 帧收发、XOR 校验、ACK、心跳失联与重试。
  *
  * 本层只做通信，不调用任何底盘/机械臂接口；任务编排由 mission 负责。
  * 命令类 API 与 Vision_Link_Update() 必须在同一任务上下文调用（MainTask02，
  * 5ms 节拍）。字节解析在 USART3 中断中完成，命令与 ACK 发送在任务上下文。
  ******************************************************************************
  */

#ifndef APP_VISION_LINK_H
#define APP_VISION_LINK_H

#include <stdint.h>

/* B/C/D 任务点；扫码等无分组命令使用 NONE */
typedef enum
{
    VISION_ZONE_B = 0,
    VISION_ZONE_C,
    VISION_ZONE_D,
    VISION_ZONE_NONE = 0xFF
} Vision_Zone_e;

/* 追踪目标：B 分物体与排爆桶两路，C/D 各一路 */
typedef enum
{
    VISION_TRACK_B_OBJECT = 0,   /* 0x22/0x23/0x24 */
    VISION_TRACK_B_BUCKET,       /* 0x25/0x26/0x27 */
    VISION_TRACK_C,              /* 0x32/0x33/0x34 */
    VISION_TRACK_D,              /* 0x42/0x43/0x44 */
    VISION_TRACK_NONE = 0xFF
} Vision_TrackTarget_e;

/* 命令类 API 返回值 */
typedef enum
{
    VISION_LINK_OK = 0,
    VISION_LINK_ERR_ARG = -1,       /* zone/target/hz 非法 */
    VISION_LINK_ERR_STATE = -2,     /* 状态不匹配，如 stop 了别的 target */
    VISION_LINK_ERR_BUSY = -3,      /* 已有 pending 或正在追踪 */
    VISION_LINK_ERR_OFFLINE = -4,   /* 链路不在线 */
    VISION_LINK_ERR_UART = -5,      /* HAL 发送失败 */
    VISION_LINK_ERR_NO_BUDGET = -6  /* 本拍发送预算耗尽 */
} Vision_Link_Ret_e;

/* 事件类型。追踪偏移不进事件，用 Vision_Link_GetTrack() 取最新值 */
typedef enum
{
    VISION_EV_NONE = 0,
    VISION_EV_SCAN_OK,          /* 0x11 */
    VISION_EV_SCAN_ERROR,       /* 0x12，value=1..6 */
    VISION_EV_SCAN_TIMEOUT,     /* 3 次发送后仍无响应 */
    VISION_EV_MIDLINE,          /* 0x21/0x31/0x41，value=0..3 */
    VISION_EV_ORDER,            /* 0x2A/0x3A/0x4A，flags bit0=1 表示 3s 兜底 */
    VISION_EV_TARGET_LOST,      /* 0x13 或首个 500ms 窗口无偏移 */
    VISION_EV_TRACK_LOST_5X,    /* 连续 5 个窗口无偏移，value=5 */
    VISION_EV_C_TARGET_DOWN,    /* 0x35 */
    VISION_EV_VISION_ERROR,     /* 未匹配 pending 的 0x12，value=错误码 */
    VISION_EV_LINK_LOST,        /* 在线 -> 离线，只报一次 */
    VISION_EV_LINK_RECOVERED,   /* 离线 -> 在线，只报一次 */
    VISION_EV_TXN_ABORTED,      /* 失联导致 pending 作废 */
    VISION_EV_TX_ERROR          /* 串口发送失败 */
} Vision_Event_e;

/* 事件原因 */
typedef enum
{
    VISION_REASON_NONE = 0,
    VISION_REASON_TIMEOUT,
    VISION_REASON_LINK_LOST,
    VISION_REASON_UART,
    VISION_REASON_CANCELLED
} Vision_Reason_e;

typedef struct
{
    uint32_t tick_ms;
    uint8_t  type;       /* Vision_Event_e */
    uint8_t  zone;       /* Vision_Zone_e */
    uint8_t  cmd;        /* 源命令或请求命令 */
    uint8_t  value;      /* 错误码 / 中位线值 / 名次 / 丢失次数 */
    uint8_t  reason;     /* Vision_Reason_e */
    uint8_t  flags;      /* bit0: ORDER 值为超时兜底 */
    uint8_t  reserved[2];
} Vision_Event_t;

/* 追踪最新值快照（工具系平面偏移，x 左 y 上，单位 mm） */
typedef struct
{
    int16_t  dx_mm;
    int16_t  dy_mm;
    uint32_t tick_ms;    /* 该帧到达时刻 */
    uint32_t seq;        /* 流帧序号 */
    uint32_t age_ms;     /* GetTrack 填充：now - tick_ms */
    uint8_t  target;     /* Vision_TrackTarget_e */
    uint8_t  valid;
} Vision_TrackSample_t;

/* 链路统计快照 */
typedef struct
{
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_overflow;
    uint32_t event_overflow;
    uint32_t err_noise;
    uint32_t err_len;
    uint32_t err_xor;
    uint32_t err_payload;
    uint32_t err_unknown;
    uint32_t err_frame_timeout;
    uint32_t late_responses;
    uint32_t err_unexpected;
    uint32_t retries;
    uint32_t tx_errors;
} Vision_LinkStats_t;

/* 初始化：注册 USART3 字节回调并启动接收。调度器启动前调用一次。 */
void Vision_Link_Init(void);

/* 每 5ms 调用一次：解析分派、ACK、重发、超时、失联、追踪看门狗 */
void Vision_Link_Update(void);

/* 链路是否在线（500ms 内收到过任意合法帧） */
uint8_t Vision_Link_IsOnline(void);

/* A 点扫码 0x10：1s x 3 次，结果事件 SCAN_OK / SCAN_ERROR / SCAN_TIMEOUT */
int Vision_Link_RequestScanCode(void);

/* 开始扫描 0x28/0x38/0x48：无应答，发送即返回 */
int Vision_Link_StartScan(Vision_Zone_e zone);

/* 请求中位线判定 0x20/0x30/0x40：500ms 无回包自动重发直到结果或取消 */
int Vision_Link_RequestMidline(Vision_Zone_e zone);

/* 取消中位线持续重发。取消成功返回 1，当前不是中位线事务返回 0。 */
uint8_t Vision_Link_CancelMidline(void);

/* 取消当前任意未决事务（扫码/中位线/顺序），不发帧。取消成功返回 1。 */
uint8_t Vision_Link_CancelPending(void);

/* 请求顺序名次 0x29/0x39/0x49：3s 无回包按 1=中兜底（flags bit0=1） */
int Vision_Link_RequestOrder(Vision_Zone_e zone);

/* 开始追踪：hz=0 使用默认 50Hz；同一时刻只允许一个 target */
int Vision_Link_TrackStart(Vision_TrackTarget_e target, uint16_t hz);

/* 停止追踪；target 与当前追踪不一致返回 ERR_STATE */
int Vision_Link_TrackStop(Vision_TrackTarget_e target);

/* 取追踪最新值；未追踪、无样本或样本年龄 >=500ms 返回 0 */
uint8_t Vision_Link_GetTrack(Vision_TrackTarget_e target, Vision_TrackSample_t *out);

/* 取一个事件，取后清除；无事件返回 0 */
uint8_t Vision_Link_TakeEvent(Vision_Event_t *ev);

/* 统计快照 */
void Vision_Link_GetStats(Vision_LinkStats_t *out);

#endif /* APP_VISION_LINK_H */
