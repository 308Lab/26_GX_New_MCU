/**
  ******************************************************************************
  * @file    oled.c
  * @brief   软件 I2C + SSD1306 128x64，5x7 ASCII 字体（大写显示）。
  *
  * 说明：SDA 为推挽输出，无法回读 ACK，因此忽略 ACK 应答。
  * 显示异常时优先检查 OLED_I2C_ADDR 与实际模块是否一致。
  ******************************************************************************
  */

#include "common/oled.h"
#include "main.h"
#include <string.h>

#define SCL_H()  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET)

/* ASCII 0x20 ~ 0x5A，每字符 5 列，bit0 为顶行 */
static const uint8_t s_font5x7[59][5] =
{
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* 0x20 ' '  */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* 0x21 '!'  */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* 0x22 '"'  */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* 0x23 '#'  */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* 0x24 '$'  */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* 0x25 '%'  */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* 0x26 '&'  */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* 0x27 '\'' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* 0x28 '('  */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* 0x29 ')'  */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* 0x2A '*'  */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* 0x2B '+'  */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* 0x2C ','  */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* 0x2D '-'  */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* 0x2E '.'  */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* 0x2F '/'  */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0x30 '0'  */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 0x31 '1'  */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 0x32 '2'  */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 0x33 '3'  */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 0x34 '4'  */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 0x35 '5'  */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 0x36 '6'  */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 0x37 '7'  */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 0x38 '8'  */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 0x39 '9'  */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* 0x3A ':'  */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* 0x3B ';'  */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* 0x3C '<'  */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* 0x3D '='  */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* 0x3E '>'  */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* 0x3F '?'  */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* 0x40 '@'  */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 0x41 'A'  */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 0x42 'B'  */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* 0x43 'C'  */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 0x44 'D'  */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 0x45 'E'  */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 0x46 'F'  */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 0x47 'G'  */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 0x48 'H'  */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 0x49 'I'  */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* 0x4A 'J'  */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 0x4B 'K'  */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 0x4C 'L'  */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 0x4D 'M'  */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 0x4E 'N'  */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 0x4F 'O'  */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 0x50 'P'  */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 0x51 'Q'  */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 0x52 'R'  */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 0x53 'S'  */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 0x54 'T'  */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 0x55 'U'  */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 0x56 'V'  */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 0x57 'W'  */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 0x58 'X'  */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* 0x59 'Y'  */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 0x5A 'Z'  */
};

static const uint8_t s_init_seq[] =
{
    0xAE,             /* display off */
    0x20, 0x00,       /* horizontal addressing */
    0xB0,             /* page start */
    0xC8,             /* COM scan remapped */
    0x00, 0x10,       /* column 0 */
    0x40,             /* start line 0 */
    0x81, 0x7F,       /* contrast */
    0xA1,             /* segment remap */
    0xA6,             /* normal display */
    0xA8, 0x3F,       /* multiplex 1/64 */
    0xA4,             /* display follows RAM */
    0xD3, 0x00,       /* display offset */
    0xD5, 0x80,       /* clock divide */
    0xD9, 0xF1,       /* pre-charge */
    0xDA, 0x12,       /* COM pins */
    0xDB, 0x40,       /* VCOM detect */
    0x8D, 0x14,       /* charge pump on */
    0xAF              /* display on */
};

static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];

static void i2c_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 30U; i++)
    {
        __NOP();
    }
}

static void i2c_start(void)
{
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_L();
    i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_H();
    i2c_delay();
}

static void i2c_write(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8U; i++)
    {
        if (b & 0x80U)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        b <<= 1;
        i2c_delay();
        SCL_H();
        i2c_delay();
        SCL_L();
        i2c_delay();
    }

    /* 第 9 个时钟：忽略 ACK（SDA 为推挽输出，无法回读） */
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    i2c_delay();
}

static void ssd1306_cmd(uint8_t cmd)
{
    i2c_start();
    i2c_write(OLED_I2C_ADDR);
    i2c_write(0x00U);
    i2c_write(cmd);
    i2c_stop();
}

void Oled_Init(void)
{
    uint32_t i;

    SCL_H();
    SDA_H();
    HAL_Delay(50);

    for (i = 0; i < sizeof(s_init_seq); i++)
    {
        ssd1306_cmd(s_init_seq[i]);
    }

    Oled_Clear();
    Oled_Flush();
}

void Oled_Clear(void)
{
    memset(s_gram, 0, sizeof(s_gram));
}

void Oled_Flush(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0; page < OLED_PAGES; page++)
    {
        ssd1306_cmd((uint8_t)(0xB0U + page));
        ssd1306_cmd(0x00U);
        ssd1306_cmd(0x10U);

        i2c_start();
        i2c_write(OLED_I2C_ADDR);
        i2c_write(0x40U);
        for (col = 0; col < OLED_WIDTH; col++)
        {
            i2c_write(s_gram[page][col]);
        }
        i2c_stop();
    }
}

void Oled_DrawStr(uint8_t x, uint8_t page, const char *s)
{
    uint8_t c;
    const uint8_t *glyph;
    uint8_t i;

    if (page >= OLED_PAGES || s == NULL)
    {
        return;
    }

    while (*s != '\0' && (uint16_t)x + 6U <= OLED_WIDTH)
    {
        c = (uint8_t)*s++;

        if (c >= (uint8_t)'a' && c <= (uint8_t)'z')
        {
            c = (uint8_t)(c - 32U);
        }
        if (c < 0x20U || c > 0x5AU)
        {
            c = (uint8_t)'?';
        }

        glyph = s_font5x7[c - 0x20U];
        for (i = 0; i < 5U; i++)
        {
            s_gram[page][x + i] = glyph[i];
        }
        s_gram[page][x + 5U] = 0x00U;
        x = (uint8_t)(x + 6U);
    }
}

static void oled_set_pixel(int16_t x, int16_t y)
{
    if ((x < 0) || (x >= (int16_t)OLED_WIDTH)
        || (y < 0) || (y >= (int16_t)OLED_HEIGHT))
    {
        return;
    }
    s_gram[y / 8][x] |= (uint8_t)(1U << (y % 8));
}

static void oled_draw_thick_point(int16_t x, int16_t y, int16_t w)
{
    int16_t dx;
    int16_t dy;

    for (dy = -w; dy <= w; dy++)
    {
        for (dx = -w; dx <= w; dx++)
        {
            oled_set_pixel((int16_t)(x + dx), (int16_t)(y + dy));
        }
    }
}

/* 把 5x7 字符按 scale 倍放大绘制到显存 */
static void oled_draw_big_glyph(const uint8_t *glyph, uint8_t x0, uint8_t y0,
                                uint8_t scale)
{
    uint8_t col;
    uint8_t row;
    uint8_t dx;
    uint8_t dy;

    for (col = 0U; col < 5U; col++)
    {
        for (row = 0U; row < 7U; row++)
        {
            if ((glyph[col] & (uint8_t)(1U << row)) == 0U)
            {
                continue;
            }
            for (dy = 0U; dy < scale; dy++)
            {
                for (dx = 0U; dx < scale; dx++)
                {
                    oled_set_pixel((int16_t)(x0 + col * scale + dx),
                                   (int16_t)(y0 + row * scale + dy));
                }
            }
        }
    }
}

void Oled_DrawBigDigit(uint8_t digit)
{
    memset(s_gram, 0, sizeof(s_gram));

    if (digit > 9U)
    {
        digit = 0U;
    }

    /* 5x7 字体放大 8 倍：单字 40x56，水平居中 */
    oled_draw_big_glyph(s_font5x7[(uint8_t)('0' - 0x20) + digit], 44U, 4U, 8U);

    Oled_Flush();
}

void Oled_DrawBigCircle(void)
{
    const int16_t cx = 64;
    const int16_t cy = 32;
    const int16_t r_in = 24;
    const int16_t r_out = 30;
    int16_t x;
    int16_t y;

    memset(s_gram, 0, sizeof(s_gram));

    for (y = 0; y < (int16_t)OLED_HEIGHT; y++)
    {
        for (x = 0; x < (int16_t)OLED_WIDTH; x++)
        {
            int16_t dx = (int16_t)(x - cx);
            int16_t dy = (int16_t)(y - cy);
            int16_t d2 = (int16_t)(dx * dx + dy * dy);

            if ((d2 >= (int16_t)(r_in * r_in)) && (d2 <= (int16_t)(r_out * r_out)))
            {
                oled_set_pixel(x, y);
            }
        }
    }

    Oled_Flush();
}

void Oled_DrawBigCross(void)
{
    const int16_t cx = 64;
    const int16_t cy = 32;
    const int16_t len = 24;
    int16_t t;

    memset(s_gram, 0, sizeof(s_gram));

    for (t = -len; t <= len; t++)
    {
        oled_draw_thick_point((int16_t)(cx + t), (int16_t)(cy + t), 3);
        oled_draw_thick_point((int16_t)(cx + t), (int16_t)(cy - t), 3);
    }

    Oled_Flush();
}

void Oled_DrawStrLine(uint8_t page, const char *s)
{
    uint8_t i;

    if (page >= OLED_PAGES)
    {
        return;
    }

    for (i = 0; i < OLED_WIDTH; i++)
    {
        s_gram[page][i] = 0x00U;
    }

    Oled_DrawStr(0, page, s);
}
