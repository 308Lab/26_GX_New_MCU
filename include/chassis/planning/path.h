/**
  ******************************************************************************
  * @file    chassis/planning/path.h
  * @brief   底盘规划层：路径段表与段调度状态机。
  *
  * 规划层不接触感知层。每段只描述“类型 + 方向 + 距离/转角”，
  * 绝对航向由控制层根据相对转角自行累加。
  *
  * 执行方式为非阻塞：Chassis_Path_Update() 每 5ms 推进一次并立即返回，
  * 跑图期间键盘扫描、调试与任务模块均可并行运行。
  ******************************************************************************
  */

#ifndef CHASSIS_PLANNING_PATH_H
#define CHASSIS_PLANNING_PATH_H

#include <stdint.h>

typedef enum
{
    PATH_STRAIGHT = 0,   /* 直行 */
    PATH_ARC,            /* 弧线转弯（主用） */
    PATH_SPIN,           /* 原地旋转（窄处备用） */
    PATH_SLOW,           /* 低速直行（减速带） */
    PATH_SCAN,           /* 识别扫描段：低速走完，倒回本段一半 */
    PATH_CALIB,          /* 陀螺仪零偏重标定：停车，等待校准完成 */
    PATH_TASK,           /* 任务点：停车，等待外部任务完成 */
    PATH_STOP,           /* 固定停顿 */
    PATH_TYPE_MAX
} Path_SegType_e;

typedef struct
{
    Path_SegType_e type;
    int8_t   dir;          /* +1 前进/左转，-1 后退/右转 */
    float    dist_mm;      /* 直行距离 / 弧长 */
    float    angle_deg;    /* SPIN / ARC 相对转角（绝对值） */
    float    radius_mm;    /* ARC 半径（绝对值） */
    float    v_mmps;       /* 0 = 使用默认巡航速度 */
    float    a_mmps2;      /* 0 = 使用默认加速度 */
    uint16_t dwell_ms;     /* STOP 等待；SCAN 的停车扫描等待 */
    uint8_t  brake;        /* 段末是否反向脉冲制动 */
    uint16_t timeout_ms;   /* 0 = 自动估算 */
    uint8_t  point_id;     /* 0 = 非任务点；'A'..'D' = 任务点 */
} Path_Seg_t;

typedef struct
{
    const char       *name;
    const Path_Seg_t *segs;
    uint16_t          count;
} Path_Table_t;

typedef enum
{
    PLAN_IDLE = 0,
    PLAN_RUNNING,
    PLAN_WAIT_TASK,
    PLAN_DONE,
    PLAN_ERROR
} Plan_State_e;

typedef enum
{
    PATH_EV_NONE = 0,
    PATH_EV_SCAN_BEGIN,     /* 扫描段开始，可发 0x28/0x38/0x48 */
    PATH_EV_SCAN_READY,     /* 扫描段走完，请视觉取图 */
    PATH_EV_ARRIVE_POINT,   /* 已到任务中心，可执行任务 */
    PATH_EV_AUX_DONE,       /* 伺服单步/辅助动作完成 */
    PATH_EV_DONE,           /* 整条路径完成 */
    PATH_EV_ERROR           /* 超时或故障 */
} Path_Event_e;

typedef struct
{
    Path_Event_e type;
    uint8_t      point_id;
    float        value;     /* 例如扫描段实际距离 */
} Path_Event_t;

int  Chassis_Path_Init(void);
int  Chassis_Path_SelectById(uint16_t id);
int  Chassis_Path_SelectByName(const char *name);
int  Chassis_Path_Start(void);
void Chassis_Path_Abort(void);
Plan_State_e Chassis_Path_GetState(uint16_t *seg_idx, uint16_t *seg_total);
uint8_t Chassis_Path_IsRunning(void);
uint8_t Chassis_Path_IsWaitingTask(void);

/* 非阻塞推进：每 5ms 调用一次，立即返回 */
void Chassis_Path_Update(void);

/* 取一次路径事件，取后清除；无事件返回 0 */
uint8_t Chassis_Path_TakeEvent(Path_Event_t *ev);

/* 外部任务完成通知：0=接受，-1=当前不在等待/点号不符 */
int Chassis_Path_NotifyTaskDone(uint8_t point_id);

/* 设置当前任务名次（0/1/2），影响扫描倒回距离：rank0 多退 150，rank2 少退 150 */
void Chassis_Path_SetRank(uint8_t rank);

/* 路径空闲时的单步移动（C 点底盘前后伺服）：完成发 PATH_EV_AUX_DONE */
int Chassis_Path_ServoStep(int16_t forward_mm, int16_t turn_deg);

#endif /* CHASSIS_PLANNING_PATH_H */
