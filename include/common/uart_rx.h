/**
  ******************************************************************************
  * @file    common/uart_rx.h
  * @brief   按 UART 实例分发的单字节中断接收。
  *
  * 各驱动注册自己的实例与字节回调，避免多个 .c 重复覆盖 HAL 弱符号。
  ******************************************************************************
  */

#ifndef COMMON_UART_RX_H
#define COMMON_UART_RX_H

#include <stdint.h>

#include "stm32f1xx_hal.h"

typedef void (*Uart_RxByteFn)(uint8_t byte);

/* 注册实例与回调。同一实例重复注册会覆盖旧回调。 */
void Uart_Rx_Register(UART_HandleTypeDef *huart, Uart_RxByteFn fn);

/* 启动单字节中断接收（注册后调用） */
void Uart_Rx_Start(UART_HandleTypeDef *huart);

#endif /* COMMON_UART_RX_H */
