#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "./SYSTEM/sys/sys.h"
 
#define FMC_FLASH_BASE      0x08000000   // FLASH的起始地址
#define FMC_FLASH_END       0x08040000   // FLASH的结束地址


#define ADDR_FLASH_SECTOR_EEPROM    0x0803E000


 /* STM32F103 扇区大小 */
#if    (0)
#define FMC_SECTOR_SIZE   1024                /* 容量小于256K的 F103, 扇区大小为1K字节 */
#else
#define FMC_SECTOR_SIZE   2048                /* 容量大于等于于256K的 F103, 扇区大小为2K字节 */
#endif

 
void stmflash_write(uint32_t waddr, uint16_t *pbuf, uint16_t length);

void stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length);

#endif

















