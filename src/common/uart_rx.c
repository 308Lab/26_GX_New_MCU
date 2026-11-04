/**
  ******************************************************************************
  * @file    common/uart_rx.c
  * @brief   UART 单字节接收分发。中断内只调用已注册的字节回调。
  ******************************************************************************
  */

#include "common/uart_rx.h"

#define UART_RX_SLOTS   (4U)

typedef struct
{
    UART_HandleTypeDef *huart;
    Uart_RxByteFn       fn;
    uint8_t             byte;
} Uart_Rx_Slot_t;

static Uart_Rx_Slot_t s_slots[UART_RX_SLOTS];

static Uart_Rx_Slot_t *uart_rx_find(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (huart == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < UART_RX_SLOTS; i++)
    {
        if (s_slots[i].huart == huart)
        {
            return &s_slots[i];
        }
    }

    return NULL;
}

void Uart_Rx_Register(UART_HandleTypeDef *huart, Uart_RxByteFn fn)
{
    Uart_Rx_Slot_t *slot;
    uint8_t i;

    if (huart == NULL || fn == NULL)
    {
        return;
    }

    slot = uart_rx_find(huart);
    if (slot == NULL)
    {
        for (i = 0U; i < UART_RX_SLOTS; i++)
        {
            if (s_slots[i].huart == NULL)
            {
                slot = &s_slots[i];
                slot->huart = huart;
                break;
            }
        }
    }

    if (slot != NULL)
    {
        slot->fn = fn;
        slot->byte = 0U;
    }
}

void Uart_Rx_Start(UART_HandleTypeDef *huart)
{
    Uart_Rx_Slot_t *slot = uart_rx_find(huart);

    if (slot != NULL && slot->fn != NULL)
    {
        (void)HAL_UART_Receive_IT(huart, &slot->byte, 1U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Uart_Rx_Slot_t *slot = uart_rx_find(huart);
    uint8_t byte;

    if (slot == NULL || slot->fn == NULL)
    {
        return;
    }

    byte = slot->byte;

    /* 先重新挂接收，再处理字节，避免下一字节到达时无缓冲 */
    (void)HAL_UART_Receive_IT(huart, &slot->byte, 1U);
    slot->fn(byte);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Uart_Rx_Slot_t *slot = uart_rx_find(huart);

    if (slot == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(huart, &slot->byte, 1U);
}
