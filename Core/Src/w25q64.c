#include "w25q64.h"

/* 片选(CS)引脚：PA4，默认高电平（不选中） */
#define W25Q64_CS_PORT    GPIOA
#define W25Q64_CS_PIN     GPIO_PIN_4
#define W25Q64_CS_LOW()   HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN, GPIO_PIN_RESET)
#define W25Q64_CS_HIGH()  HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;

/* 通过 SPI 收发一个字节 */
static uint8_t W25Q64_SPI_SwapByte(uint8_t byte)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, 1, 100);
    return rx;
}

/* 写使能 */
static void W25Q64_WriteEnable(void)
{
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_WRITE_ENABLE);
    W25Q64_CS_HIGH();
}

/* 读状态寄存器 1 */
static uint8_t W25Q64_ReadStatus(void)
{
    uint8_t status;
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_READ_STATUS1);
    status = W25Q64_SPI_SwapByte(0xFF);
    W25Q64_CS_HIGH();
    return status;
}

/* 等待忙标志(BUSY, bit0)清除 */
static void W25Q64_WaitBusy(void)
{
    while (W25Q64_ReadStatus() & 0x01);
}

/* 读 JEDEC ID：W25Q64 应为 0xEF4017 (Winbond) */
uint32_t W25Q64_ReadID(void)
{
    uint8_t id[3];
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_READ_ID);
    id[0] = W25Q64_SPI_SwapByte(0xFF);   /* 厂商 0xEF */
    id[1] = W25Q64_SPI_SwapByte(0xFF);   /* 内存类型 0x40 */
    id[2] = W25Q64_SPI_SwapByte(0xFF);   /* 容量 0x17 */
    W25Q64_CS_HIGH();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

/* 初始化：读 ID 校验 */
uint8_t W25Q64_Init(void)
{
    return (W25Q64_ReadID() == 0xEF4017) ? 1 : 0;
}

/* 读数据 */
void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_READ_DATA);
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 8) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)(addr & 0xFF));
    for (uint32_t i = 0; i < len; i++)
    {
        buf[i] = W25Q64_SPI_SwapByte(0xFF);
    }
    W25Q64_CS_HIGH();
}

/* 写数据（页编程，自动处理跨页） */
void W25Q64_WriteData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t page_remain = W25Q64_PAGE_SIZE - (addr % W25Q64_PAGE_SIZE);
    if (len < page_remain)
    {
        page_remain = len;
    }

    while (1)
    {
        W25Q64_WriteEnable();
        W25Q64_CS_LOW();
        W25Q64_SPI_SwapByte(W25Q64_CMD_PAGE_PROGRAM);
        W25Q64_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFF));
        W25Q64_SPI_SwapByte((uint8_t)((addr >> 8) & 0xFF));
        W25Q64_SPI_SwapByte((uint8_t)(addr & 0xFF));
        for (uint32_t i = 0; i < page_remain; i++)
        {
            W25Q64_SPI_SwapByte(buf[i]);
        }
        W25Q64_CS_HIGH();
        W25Q64_WaitBusy();

        if (len == page_remain)
        {
            break;
        }
        else
        {
            buf += page_remain;
            addr += page_remain;
            len -= page_remain;
            page_remain = (len > W25Q64_PAGE_SIZE) ? W25Q64_PAGE_SIZE : len;
        }
    }
}

/* 扇区擦除(4KB) */
void W25Q64_SectorErase(uint32_t addr)
{
    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_SECTOR_ERASE);
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 8) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)(addr & 0xFF));
    W25Q64_CS_HIGH();
    W25Q64_WaitBusy();
}

/* 块擦除(64KB) */
void W25Q64_BlockErase(uint32_t addr)
{
    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_BLOCK_ERASE);
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)((addr >> 8) & 0xFF));
    W25Q64_SPI_SwapByte((uint8_t)(addr & 0xFF));
    W25Q64_CS_HIGH();
    W25Q64_WaitBusy();
}

/* 整片擦除 */
void W25Q64_ChipErase(void)
{
    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    W25Q64_SPI_SwapByte(W25Q64_CMD_CHIP_ERASE);
    W25Q64_CS_HIGH();
    W25Q64_WaitBusy();
}
