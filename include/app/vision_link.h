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

/* 命令类 API 返回值 */
typedef enum
{
    VISION_LINK_OK = 0,
    VISION_LINK_ERR_ARG = -1,       /* zone/sel 非法 */
    VISION_LINK_ERR_STATE = -2,     /* 状态不匹配，如 stop 了别的 target */
    VISION_LINK_ERR_BUSY = -3,      /* 已有 pending 或正在追踪 */
    VISION_LINK_ERR_OFFLINE = -4,   /* 链路不在线 */
    VISION_LINK_ERR_UART = -5,      /* HAL 发送失败 */
    VISION_LINK_ERR_NO_BUDGET = -6  /* 本拍发送预算耗尽 */
} Vision_Link_Ret_e;

/* v1.6 §5.3：0x21/0x31/0x41 的中位线偏移量是 int16 大端千分比（+ = 目标在画面中线右侧）。
 * 这个哨兵值表示「未检出」。不能用 0 表示没看见 —— 0 是合法的「正好居中」。 */
#define VISION_LINK_MIDLINE_UNDETECTED       ((int16_t)0x8000)

/* 事件类型 */
typedef enum
{
    VISION_EV_NONE = 0,
    VISION_EV_SCAN_OK,          /* 0x11 */
    VISION_EV_SCAN_ERROR,       /* 0x12，value=1..6 */
    VISION_EV_SCAN_TIMEOUT,     /* 3 次发送后仍无响应 */
    VISION_EV_MIDLINE,          /* 0x21/0x31/0x41，偏移量在 value16（千分比，+ = 目标在中线右侧，
                                 *  VISION_LINK_MIDLINE_UNDETECTED 表示未检出）；value 恒 0 */
    VISION_EV_ORDER,            /* 0x2A/0x3A/0x4A，flags bit0=1 表示 3s 兜底 */
    VISION_EV_TARGET_LOST,      /* 0x13（v1.6 起不再挂本地追踪上下文，只作事件上报） */
    VISION_EV_TRACK_LOST_5X,    /* 已废弃：v1.6 取消追踪帧后不会再产生，保留枚举避免破坏旧代码 */
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
    int16_t  value16;    /* MIDLINE：int16 千分比偏移量，0x8000 = 未检出 */
} Vision_Event_t;

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

/* 每 5ms 调用一次：解析分派、ACK、重发、超时、失联 */
void Vision_Link_Update(void);

/* 链路是否在线（500ms 内收到过任意合法帧） */
uint8_t Vision_Link_IsOnline(void);

/* A 点扫码 0x10：1s x 3 次，结果事件 SCAN_OK / SCAN_ERROR / SCAN_TIMEOUT */
int Vision_Link_RequestScanCode(void);

/* 开始扫描 0x28/0x38/0x48：无应答，发送即返回 */
int Vision_Link_StartScan(Vision_Zone_e zone);

/* 请求中位线判定 0x20/0x30/0x40（v1.6 §4.3 带 1 字节 sel，0 = 不指定）：500ms 无回包自动重发直到结果或取消；
 * 2s 仍无结果则自动超时（事件 VISION_EV_VISION_ERROR + reason=TIMEOUT）并清事务，
 * 调用方不必依赖「一定有人 CancelMidline」。 */
int Vision_Link_RequestMidline(Vision_Zone_e zone, uint8_t sel);

/* 取消中位线持续重发。取消成功返回 1，当前不是中位线事务返回 0。 */
uint8_t Vision_Link_CancelMidline(void);

/* 取消当前任意未决事务（扫码/中位线/顺序），不发帧。取消成功返回 1。 */
uint8_t Vision_Link_CancelPending(void);

/* 请求顺序名次 0x29/0x39/0x49（v1.6 §4.3 带 1 字节 sel，0 = 不指定）：3s 无回包按 1=中兜底（flags bit0=1） */
int Vision_Link_RequestOrder(Vision_Zone_e zone, uint8_t sel);

/* 取一个事件，取后清除；无事件返回 0 */
uint8_t Vision_Link_TakeEvent(Vision_Event_t *ev);

/* 统计快照 */
void Vision_Link_GetStats(Vision_LinkStats_t *out);

#endif /* APP_VISION_LINK_H */
