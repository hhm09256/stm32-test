#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "w25q64.h"

/* USB CDC 串口输出开关。
   USB 设备库(Middlewares/USB_DEVICE)文件缺失，需用 CubeMX 重新生成后，
   取消下面这行注释即可启用 USB 虚拟串口输出。 */
// #define USB_CDC_ENABLED

#ifdef USB_CDC_ENABLED
#include "usbd_cdc_if.h"
#endif

/* ADC 句柄：由 main.c 的 MX_ADC1_Init() 初始化 */
extern ADC_HandleTypeDef hadc1;
/* I2C 句柄：由 main.c 的 MX_I2C1_Init() 初始化，用于驱动 OLED */
extern I2C_HandleTypeDef hi2c1;

/* SSD1306 OLED（0.96 寸 128x64，I2C） */
#define OLED_ADDR      (0x3C << 1)       /* 7 位地址 0x3C，HAL 需要左移 1 位 */
#define OLED_WIDTH     128
#define OLED_HEIGHT    64
#define OLED_PAGES     (OLED_HEIGHT / 8) /* 页数（每页 8 行） */
#define OLED_I2C_TIMEOUT  100            /* I2C 单次传输超时(ms) */

/* 有源蜂鸣器：信号脚接 PB5，低电平触发（响）。 */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_PIN     GPIO_PIN_5
#define BUZZER_ON()         HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET)
#define BUZZER_OFF()        HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET)

/* 分压电路：3.3V —— 10kΩ —— PAx —— 传感器 —— GND（下拉分压，传感器在低端） */
#define VREF_MV        3300      /* 参考电压 3.3V，单位 mV */
#define ADC_MAX        4095      /* 12 位 ADC 满量程 */
#define DIVIDER_RES    10000.0f  /* 分压固定电阻 10kΩ */

/* NTC 热敏电阻参数：10kΩ @25°C，B 值按你的 NTC 型号调整 */
#define NTC_R25        10000.0f
#define NTC_B          3435.0f
#define NTC_T0_K       298.15f   /* 25°C = 298.15 K */

/* ADC 采样参数：单次转换超时(ms)、每通道求平均的采样次数 */
#define ADC_TIMEOUT_MS   100
#define ADC_AVG_SAMPLES  8

/* 报警阈值：温度 > 35℃ 或 光照 > 85% → 蜂鸣器响 */
#define ALARM_TEMP_C     35.0f
#define ALARM_LIGHT_PCT  85

/* 自定义 print()：把格式化文本通过 USB 虚拟串口(CDC) 发到电脑 */
static void print(const char *fmt, ...)
{
#ifdef USB_CDC_ENABLED
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t *)buf, (uint16_t)strlen(buf));
#else
    (void)fmt;   /* USB 未启用时忽略输出 */
#endif
}

/* 读取指定 ADC 通道的原始值；转换失败（配置失败或超时）返回 false，且不改 *out。
   注意：STM32F1 扫描模式下 EOC 标志只在整组转换结束时才置位，DR 寄存器
   只保留最后一个通道的值，多通道轮询会读错；所以这里用“单通道+切换”的方式。 */
static bool read_adc_channel(uint32_t channel, uint32_t *out)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)   /* 切换到要读的通道 */
    {
        return false;
    }

    bool ok = (HAL_ADC_Start(&hadc1) == HAL_OK)
              && (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) == HAL_OK);
    if (ok)
    {
        *out = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return ok;
}

/* 对指定通道连续采样 ADC_AVG_SAMPLES 次取平均，减小噪声；任一次转换失败即返回 false */
static bool read_adc_average(uint32_t channel, uint32_t *out)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < ADC_AVG_SAMPLES; i++)
    {
        uint32_t value;
        if (!read_adc_channel(channel, &value))
        {
            return false;
        }
        sum += value;
    }

    *out = sum / ADC_AVG_SAMPLES;
    return true;
}

/* ADC 原始值 → 电压（mV） */
static uint32_t adc_to_mv(uint32_t raw)
{
    return (raw * VREF_MV) / ADC_MAX;
}

/* 光敏：ADC 原始值 → 亮度百分比(0~100%)
   分压电路（下拉分压，光敏在 PAx 与 GND 之间）：
     光越强 → 光敏电阻越小 → 电压越低 → 百分比越高
     即 V=0（最亮）→ 100%，V=VREF（最暗）→ 0% */
static uint32_t adc_to_percent(uint32_t raw)
{
    return 100 - (raw * 100) / ADC_MAX;
}

/* 热敏：电压(mV) → 温度(°C)
   分压电路（下拉分压，NTC 在 PAx 与 GND 之间）：
     V = 3.3 * R_ntc / (10k + R_ntc)   →   R_ntc = 10k * V / (3.3 - V)
   温度：
     T(K) = 1 / (1/T0 + ln(R_ntc / R25) / B) */
static float ntc_to_celsius(uint32_t mv)
{
    if (mv == 0 || mv >= VREF_MV)
    {
        return -999.0f;                          /* 电压异常，返回无效值 */
    }

    float v = mv / 1000.0f;                       /* 电压 V */
    float vref = VREF_MV / 1000.0f;               /* 参考电压 V */
    float r_ntc = DIVIDER_RES * v / (vref - v);   /* NTC 当前阻值（NTC 在低端） */
    float t_k = 1.0f / (1.0f / NTC_T0_K + logf(r_ntc / NTC_R25) / NTC_B);

    return t_k - 273.15f;                         /* 转摄氏度 */
}

/* ==================== W25Q64 上电自检 ==================== */

/* 是否在上电时做一次“擦-写-读-比对”自检。
   注意：自检会擦除 W25Q64 的最后一个 4KB 扇区（地址 0x7FF000），
   如果以后把参数/日志保存在该扇区，请把这里改成 0。 */
#define W25Q64_SELFTEST_ENABLED   1
#define W25Q64_SELFTEST_ADDR      (W25Q64_FLASH_SIZE - W25Q64_SECTOR_SIZE)

/* 自检结果 */
#define W25Q64_STATE_NO_ID        0   /* 读不到 JEDEC ID：没插片子/接线错误 */
#define W25Q64_STATE_OK           1   /* ID 正确且读写一致 */
#define W25Q64_STATE_BAD_DATA     2   /* ID 正确但读回数据不符 */

static uint8_t  w25q64_state = W25Q64_STATE_NO_ID;
static uint32_t w25q64_id    = 0;

static void w25q64_selftest(void)
{
    /* 自检用的固定测试图案 */
    static uint8_t pattern[8] = {0xA5, 0x5A, 0x01, 0xFE, 0x12, 0x34, 0x56, 0x78};
    uint8_t readback[8];

    w25q64_id = W25Q64_ReadID();
    if (w25q64_id != 0xEF4017)          /* 0xEF4017 = Winbond W25Q64 */
    {
        w25q64_state = W25Q64_STATE_NO_ID;
        return;
    }

#if W25Q64_SELFTEST_ENABLED
    W25Q64_SectorErase(W25Q64_SELFTEST_ADDR);            /* 擦 4KB */
    W25Q64_WriteData(W25Q64_SELFTEST_ADDR, pattern, sizeof(pattern));
    W25Q64_ReadData(W25Q64_SELFTEST_ADDR, readback, sizeof(readback));

    w25q64_state = (memcmp(readback, pattern, sizeof(pattern)) == 0)
                   ? W25Q64_STATE_OK
                   : W25Q64_STATE_BAD_DATA;
#else
    w25q64_state = W25Q64_STATE_OK;     /* 只做 ID 校验 */
#endif
}

/* ==================== SSD1306 OLED 驱动 ==================== */

static uint8_t oled_buf[OLED_WIDTH * OLED_HEIGHT / 8];   /* 页式帧缓冲：8 页 × 128 列 */

/* 发送一条命令 */
static void oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};       /* 控制字节 0x00 表示后面是命令 */
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, OLED_I2C_TIMEOUT);
}

/* 点亮一个像素点 */
static void oled_set_pixel(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        return;
    }
    oled_buf[(y / 8) * OLED_WIDTH + x] |= (uint8_t)(1 << (y % 8));
}

/* 清空帧缓冲 */
static void oled_clear(void)
{
    memset(oled_buf, 0, sizeof(oled_buf));
}

/* 把帧缓冲整体刷新到屏幕（定义在下方） */
static void oled_update(void);

/* 初始化 SSD1306 */
static void oled_init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,          /* 关闭显示 */
        0xD5, 0x80,    /* 时钟分频 */
        0xA8, 0x3F,    /* 复用率 64 */
        0xD3, 0x00,    /* 显示偏移 */
        0x40,          /* 起始行 0 */
        0x8D, 0x14,    /* 打开电荷泵 */
        0x20, 0x00,    /* 内存寻址：水平模式 */
        0xA1,          /* 段重映射 */
        0xC8,          /* COM 扫描方向 */
        0xDA, 0x12,    /* COM 引脚配置 */
        0x81, 0xCF,    /* 对比度 */
        0xD9, 0xF1,    /* 预充电周期 */
        0xDB, 0x40,    /* VCOMH 电平 */
        0xA4,          /* 显示 RAM 内容 */
        0xA6,          /* 正常显示（非反色） */
        0xAF,          /* 打开显示 */
    };
    for (uint32_t i = 0; i < sizeof(init_cmds); i++)
    {
        oled_write_cmd(init_cmds[i]);
    }
    oled_clear();
    oled_update();
}

/* 把帧缓冲整体刷新到屏幕（逐页发送） */
static void oled_update(void)
{
    uint8_t buf[OLED_WIDTH + 1];             /* 一页的“控制字节 + 128 列数据” */
    buf[0] = 0x40;                           /* 控制字节 0x40 表示后面是数据 */

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        oled_write_cmd((uint8_t)(0xB0 | page));  /* 设置页地址 */
        oled_write_cmd(0x00);                    /* 列地址低 4 位 */
        oled_write_cmd(0x10);                    /* 列地址高 4 位 */

        memcpy(buf + 1, &oled_buf[page * OLED_WIDTH], OLED_WIDTH);
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, OLED_WIDTH + 1,
                                OLED_I2C_TIMEOUT);
    }
}

/* 8x16 ASCII 字库（只包含本程序用到的字符），每字模 16 字节、行优先、bit7 在最左 */
static const uint8_t font8x16[][16] = {
    /* 0: 空格 */ {0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00},
    /* 1: '0' */  {0x00,0x00, 0x3C,0x7E,0xE7,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C, 0x00,0x00},
    /* 2: '1' */  {0x00,0x00, 0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x7E, 0x00,0x00},
    /* 3: '2' */  {0x00,0x00, 0x3C,0x7E,0xC3,0x03,0x03,0x06,0x0C,0x18,0x30,0x60,0xFF,0xFF, 0x00,0x00},
    /* 4: '3' */  {0x00,0x00, 0x3C,0x7E,0xC3,0x03,0x03,0x1E,0x1E,0x03,0x03,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 5: '4' */  {0x00,0x00, 0x06,0x0E,0x1E,0x36,0x66,0xC6,0xFF,0xFF,0x06,0x06,0x06,0x06, 0x00,0x00},
    /* 6: '5' */  {0x00,0x00, 0xFF,0xFF,0xC0,0xC0,0xFC,0xFE,0x07,0x03,0x03,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 7: '6' */  {0x00,0x00, 0x1E,0x3E,0x70,0xE0,0xC0,0xFC,0xFE,0xC3,0xC3,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 8: '7' */  {0x00,0x00, 0xFF,0xFF,0x03,0x03,0x06,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30, 0x00,0x00},
    /* 9: '8' */  {0x00,0x00, 0x3C,0x7E,0xC3,0xC3,0xC3,0x7E,0x7E,0xC3,0xC3,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 10: '9' */ {0x00,0x00, 0x3C,0x7E,0xC3,0xC3,0xC3,0x7F,0x3F,0x03,0x03,0x06,0x0C,0x38, 0x00,0x00},
    /* 11: '.' */ {0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x3C,0x3C,0x18, 0x00,0x00},
    /* 12: '%' */ {0x00,0x00, 0x63,0x66,0x0C,0x0C,0x18,0x18,0x30,0x30,0x60,0x60,0xC6,0xC3, 0x00,0x00},
    /* 13: 'C' */ {0x00,0x00, 0x1E,0x3F,0x73,0xE1,0xC0,0xC0,0xC0,0xC0,0xE1,0x73,0x3F,0x1E, 0x00,0x00},
    /* 14: 'L' */ {0x00,0x00, 0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF, 0x00,0x00},
    /* 15: 'i' */ {0x00,0x00, 0x18,0x18,0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18, 0x00,0x00},
    /* 16: 'g' */ {0x00,0x00, 0x3C,0x7E,0xC6,0xC6,0xC6,0xC6,0xC6,0x7E,0x3E,0x06,0xC6,0x7C, 0x00,0x00},
    /* 17: 'h' */ {0x00,0x00, 0xC0,0xC0,0xC0,0xC0,0xC0,0xFC,0xFE,0xC3,0xC3,0xC3,0xC3,0xC3, 0x00,0x00},
    /* 18: 't' */ {0x00,0x00, 0x18,0x18,0x18,0x18,0x18,0x7E,0x7E,0x18,0x18,0x18,0x18,0x1C, 0x00,0x00},
    /* 19: 'T' */ {0x00,0x00, 0xFF,0xFF,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18, 0x00,0x00},
    /* 20: 'e' */ {0x00,0x00, 0x3C,0x7E,0xC3,0xC3,0xFF,0xFF,0xC0,0xC0,0xC3,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 21: 'm' */ {0x00,0x00, 0x00,0x00,0x00,0x00,0xC6,0xEF,0xFF,0xDB,0xDB,0xDB,0xDB,0xDB, 0x00,0x00},
    /* 22: 'p' */ {0x00,0x00, 0x00,0x00,0x00,0x00,0xFC,0xFE,0xC3,0xC3,0xC3,0xFE,0xFC,0xC0, 0x00,0x00},
    /* 23: ':' */ {0x00,0x00, 0x00,0x00,0x18,0x3C,0x3C,0x18,0x00,0x00,0x18,0x3C,0x3C,0x18, 0x00,0x00},
    /* 24: '-' */ {0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00, 0x00,0x00},
    /* 25: 'F' */ {0x00,0x00, 0xFF,0xC0,0xC0,0xC0,0xF8,0xF8,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0, 0x00,0x00},
    /* 26: 'A' */ {0x00,0x00, 0x18,0x18,0x3C,0x3C,0x66,0x66,0xC3,0xFF,0xFF,0xC3,0xC3,0xC3, 0x00,0x00},
    /* 27: 'S' */ {0x00,0x00, 0x3E,0x7F,0xE0,0xC0,0x7C,0x3E,0x07,0x03,0xC3,0xE6,0x7E,0x3C, 0x00,0x00},
    /* 28: 'H' */ {0x00,0x00, 0xC3,0xC3,0xC3,0xC3,0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3, 0x00,0x00},
    /* 29: 'O' */ {0x00,0x00, 0x3C,0x7E,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0x7E,0x3C, 0x00,0x00},
    /* 30: 'K' */ {0x00,0x00, 0xC3,0xC6,0xCC,0xD8,0xF0,0xF0,0xD8,0xCC,0xC6,0xC3,0xC3,0xC3, 0x00,0x00},
    /* 31: 'E' */ {0x00,0x00, 0xFF,0xC0,0xC0,0xC0,0xF8,0xF8,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF, 0x00,0x00},
    /* 32: 'R' */ {0x00,0x00, 0xFC,0xC6,0xC6,0xC6,0xC6,0xFC,0xF8,0xCC,0xC6,0xC3,0xC3,0xC3, 0x00,0x00},
};

/* 字符 → 字库下标；不认识的字符用空格代替 */
static uint8_t font_index(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(1 + (c - '0'));
    switch (c)
    {
        case ' ': return 0;
        case '.': return 11;
        case '%': return 12;
        case 'C': return 13;
        case 'L': return 14;
        case 'i': return 15;
        case 'g': return 16;
        case 'h': return 17;
        case 't': return 18;
        case 'T': return 19;
        case 'e': return 20;
        case 'm': return 21;
        case 'p': return 22;
        case ':': return 23;
        case '-': return 24;
        case 'F': return 25;
        case 'A': return 26;
        case 'S': return 27;
        case 'H': return 28;
        case 'O': return 29;
        case 'K': return 30;
        case 'E': return 31;
        case 'R': return 32;
        default:  return 0;
    }
}

/* 在 (x, y) 处画一个 8x16 字符 */
static void oled_draw_char(uint8_t x, uint8_t y, char c)
{
    const uint8_t *glyph = font8x16[font_index(c)];
    for (uint8_t row = 0; row < 16; row++)
    {
        uint8_t bits = glyph[row];
        for (uint8_t col = 0; col < 8; col++)
        {
            if (bits & (0x80 >> col))
            {
                oled_set_pixel((uint8_t)(x + col), (uint8_t)(y + row));
            }
        }
    }
}

/* 在 (x, y) 处显示一行字符串（8x16 字体） */
static void oled_show_str(uint8_t x, uint8_t y, const char *str)
{
    while (*str)
    {
        oled_draw_char(x, y, *str++);
        x += 8;
    }
}

/* 在指定行显示一行格式化文本，例如 oled_show_line(24, "Temp : %sC", s) */
static void oled_show_line(uint8_t y, const char *fmt, ...)
{
    char line[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    oled_show_str(0, y, line);
}

/* 温度(°C) → "±整数.1位小数" 文本（不含单位），避免依赖 printf 的 %f 浮点支持。
   必须先四舍五入到“十分位”再拆分，否则 34.98℃ 会被显示成 34.10C
   （小数位单独进位到 10）；负号单独补，否则 -0.5℃ 会因整数部分为 0 丢号。 */
static void format_temp(char *buf, size_t len, float temp_c)
{
    int tenths = (int)lroundf(temp_c * 10.0f);
    int frac = tenths % 10;
    if (frac < 0)
    {
        frac = -frac;                        /* -34.5℃ → 小数位取 5，而不是 -5 */
    }

    snprintf(buf, len, "%s%d.%d", (temp_c < 0.0f) ? "-" : "",
             (tenths < 0 ? -tenths : tenths) / 10, frac);
}

/* ==================== 应用主循环 ==================== */

/* 一次采样的结果；temp_str 预先格式化好，供 OLED 与串口输出共用 */
typedef struct
{
    uint32_t light_pct;      /* 亮度 0~100 % */
    float    temp_c;         /* 温度 °C，读数无效时为 -999 */
    char     temp_str[16];   /* temp_c 的显示形式，如 "-12.3"（不含单位） */
} sensor_reading_t;

/* 读一次两个通道的传感器并填入 out；读取失败时 out 保持不变 */
static void sensors_read(sensor_reading_t *out)
{
    uint32_t raw_light, raw_temp;

    if (!read_adc_average(ADC_CHANNEL_0, &raw_light) ||   /* 光敏：PA0 */
        !read_adc_average(ADC_CHANNEL_1, &raw_temp))      /* 热敏：PA1 */
    {
        return;
    }

    out->light_pct = adc_to_percent(raw_light);
    out->temp_c    = ntc_to_celsius(adc_to_mv(raw_temp));
    format_temp(out->temp_str, sizeof(out->temp_str), out->temp_c);
}

/* W25Q64 自检结果 → 短文本。按 W25Q64_STATE_* 的取值顺序索引：
   NO_ID(0) = "--"（读不到 ID）, OK(1), BAD_DATA(2) = "ER"（读写不符） */
static const char *flash_state_text(void)
{
    static const char *const text[] = {"--", "OK", "ER"};
    return text[w25q64_state];
}

/* 把当前读数画到 OLED 的三行上 */
static void oled_render(const sensor_reading_t *s)
{
    oled_clear();
    oled_show_line(0,  "Light: %lu%%", (unsigned long)s->light_pct);
    oled_show_line(24, "Temp : %sC", s->temp_str);
    oled_show_line(48, "FLASH: %s", flash_state_text());
    oled_update();
}

/* 报警：温度 > 35℃ 或 光照 > 85% → 蜂鸣器响；低于阈值 → 关闭 */
static void alarm_update(const sensor_reading_t *s)
{
    if (s->temp_c > ALARM_TEMP_C || s->light_pct > ALARM_LIGHT_PCT)
    {
        BUZZER_ON();
    }
    else
    {
        BUZZER_OFF();
    }
}

void project_main(void)
{
    oled_init();

    /* 上电自检 W25Q64：先读 JEDEC ID，ID 正确再做一次擦-写-读-比对 */
    w25q64_selftest();
    print("W25Q64 ID=0x%06lX state=%d\r\n", w25q64_id, (int)w25q64_state);

    sensor_reading_t s = {0};

    while (1)
    {
        sensors_read(&s);        /* 读取失败时沿用上一次的有效值，不跳变到 0 */

        print("Light: %lu%% | Temp: %s C\r\n",
              (unsigned long)s.light_pct, s.temp_str);

        oled_render(&s);
        alarm_update(&s);

        HAL_Delay(500);
    }
}
