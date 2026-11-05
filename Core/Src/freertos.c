/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common/fault.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW = 2 时由内核调用）。
  *
  * 审计 H8：原来只 bkpt + 关中断死等，没有调试器时现场什么也留不下。
  * 现在把出错任务名和故障码写进 BKP 备份寄存器再停机：IWDG 启动过就在
  * ≤2s 内复位，下电重启后 OLED 首页会显示 "FAULT1 <任务名>"。
  * 这里只做寄存器操作，不再申请栈（出错任务的栈已经越界）。
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;

  Fault_Record(FAULT_CODE_STACK_OVERFLOW, (const char *)pcTaskName);
  Fault_Halt();
}

/**
  * @brief 动态内存分配失败钩子（configUSE_MALLOC_FAILED_HOOK = 1）。
  *        一般是 heap_4 不够用/申请了超大块，记录后停机。
  */
void vApplicationMallocFailedHook(void)
{
  Fault_Record(FAULT_CODE_MALLOC_FAILED, NULL);
  Fault_Halt();
}
/* USER CODE END Application */

