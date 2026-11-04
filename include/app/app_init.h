/**
  ******************************************************************************
  * @file    app/app_init.h
  * @brief   全局初始化编排。
  ******************************************************************************
  */

#ifndef APP_APP_INIT_H
#define APP_APP_INIT_H

/* 在 osKernelInitialize() 之前调用：外设与各子系统初始化 */
void App_Init(void);

/* 在 osKernelInitialize() 之后调用：创建各子系统的静态队列 */
void App_InitRtos(void);

#endif /* APP_APP_INIT_H */
