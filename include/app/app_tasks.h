/**
  ******************************************************************************
  * @file    app/app_tasks.h
  * @brief   5 个 RTOS 任务的真正执行体。
  *
  * 任务体只负责节拍与编排，不含控制逻辑，也不接触任何底盘内部结构。
  ******************************************************************************
  */

#ifndef APP_APP_TASKS_H
#define APP_APP_TASKS_H

void App_DefaultTask(void *argument);
void App_MainTask(void *argument);
void App_CarTask(void *argument);
void App_EncTask(void *argument);
void App_OledTask(void *argument);
void App_ArmTask(void *argument);

#endif /* APP_APP_TASKS_H */
