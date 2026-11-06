/**
  ******************************************************************************
  * @file    oled.h
  * @brief   软件 I2C OLED 显示（SSD1306 128x64）。
  *
  * 引脚：PB8=SCL / PB9=SDA，已在 main.h 中定义为 GPIO_OUTPUT_PP。
  * 不使用 printf，避免 512B 任务栈溢出与 __io_putchar 未定义导致的 HardFault。
  ******************************************************************************
  */

#ifndef COMMON_OLED_H
#define COMMON_OLED_H

#include <stdint.h>

#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR   (0x78U)   /* 7 位地址 0x3C 左移一位 */
#endif

#define OLED_WIDTH      (128U)
#define OLED_HEIGHT     (64U)
#define OLED_PAGES      (OLED_HEIGHT / 8U)   /* 8 页，每页 8 行像素 */
#define OLED_LINE_MAX   (21U)                /* 6x8 字体，128/6 ≈ 21 字符 */

void Oled_Init(void);
void Oled_Clear(void);
void Oled_Flush(void);

/* 在指定页（0~7）与像素列（0~127）绘制 6x8 ASCII 字符串 */
void Oled_DrawStr(uint8_t x, uint8_t page, const char *s);

/* 整行显示，先补空格清到行尾 */
void Oled_DrawStrLine(uint8_t page, const char *s);

/* 全屏居中显示大圆圈 / 大叉，并立即刷新 */
void Oled_DrawBigCircle(void);
void Oled_DrawBigCross(void);

/* 全屏居中显示一个大数字（0~9） */
void Oled_DrawBigDigit(uint8_t digit);

#endif /* COMMON_OLED_H */
