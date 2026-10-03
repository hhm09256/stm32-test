#ifndef __W25Q64_H
#define __W25Q64_H

#include "main.h"

/* W25Q64: 8MB (64Mbit) SPI NOR Flash */
#define W25Q64_FLASH_SIZE     (8 * 1024 * 1024)   /* 8MB 总容量 */
#define W25Q64_SECTOR_SIZE    4096                  /* 4KB 扇区 */
#define W25Q64_BLOCK_SIZE     65536                 /* 64KB 块 */
#define W25Q64_PAGE_SIZE      256                   /* 256B 页 */

/* 指令集 */
#define W25Q64_CMD_WRITE_ENABLE    0x06
#define W25Q64_CMD_WRITE_DISABLE   0x04
#define W25Q64_CMD_READ_STATUS1    0x05
#define W25Q64_CMD_READ_DATA       0x03
#define W25Q64_CMD_PAGE_PROGRAM    0x02
#define W25Q64_CMD_SECTOR_ERASE    0x20
#define W25Q64_CMD_BLOCK_ERASE     0xD8
#define W25Q64_CMD_CHIP_ERASE      0xC7
#define W25Q64_CMD_READ_ID         0x9F

uint8_t  W25Q64_Init(void);                              /* 读 ID 校验，成功返回 1 */
uint32_t W25Q64_ReadID(void);                            /* 读 JEDEC ID，W25Q64 应为 0xEF4017 */
void     W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
void     W25Q64_WriteData(uint32_t addr, uint8_t *buf, uint32_t len);
void     W25Q64_SectorErase(uint32_t addr);              /* 4KB */
void     W25Q64_BlockErase(uint32_t addr);               /* 64KB */
void     W25Q64_ChipErase(void);

#endif /* __W25Q64_H */
