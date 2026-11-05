/**
  ******************************************************************************
  * @file    app/mission.h
  * @brief   正式模式任务层：A/B/C/D 编排、视觉事件消费、失败降级。
  *
  * 必须在与 Vision_Link_Update() 相同的任务上下文调用。
  * 路径事件与视觉事件的唯一消费者。
  ******************************************************************************
  */

#ifndef APP_MISSION_H
#define APP_MISSION_H

#include <stdint.h>

typedef enum
{
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_PREP,
    MISSION_STATE_A_TRAVEL,
    MISSION_STATE_A_TASK,
    MISSION_STATE_TO_B,
    MISSION_STATE_B_TASK,
    MISSION_STATE_TO_C,
    MISSION_STATE_C_TASK,
    MISSION_STATE_TO_D,
    MISSION_STATE_D_TASK,
    MISSION_STATE_DEMO,      /* A 扫码失败，交给演示流程跑完 B/C/D */
    MISSION_STATE_RETURN,
    MISSION_STATE_DEGRADED,
    MISSION_STATE_DONE,
    MISSION_STATE_ERROR,
    MISSION_STATE_ABORT
} Mission_State_e;

void Mission_Init(void);

/* 每 5ms 调用：事件泵 + 状态机 */
void Mission_Update(void);

/* 启动/中止 */
int  Mission_Start(void);       /* 完整 A→B→C→D */
int  Mission_StartToA(void);    /* 只跑到 A 点扫码完成即停 */
void Mission_RequestAbort(void);

uint8_t         Mission_IsActive(void);

/* 1 = mission 消费路径/视觉事件；0 = 演示流程接管 */
uint8_t         Mission_OwnsPath(void);

Mission_State_e Mission_GetState(void);
uint8_t         Mission_GetZone(void);
uint8_t         Mission_GetRank(void);

#endif /* APP_MISSION_H */
