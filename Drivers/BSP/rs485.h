/**
 ****************************************************************************************************
 * @file        rs485.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       RS485 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200424
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __RS485_H
#define __RS485_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/



/* RS485 命令 参考modbus报文形式，数据长度固定两个字节*/
typedef struct
{
    volatile uint8_t      slave;        /* 从机地址 */
    volatile uint8_t      command;      /* 读写命令 */
    volatile uint16_t      w_rpm;       /* 写转速 */
    volatile uint16_t      w_current;   /* 写电流 */
    volatile uint16_t      r_rpm;       /* 读转速 */
    volatile uint16_t      r_current;   /* 读电流 */
    volatile uint16_t      r_vbus;      /* 读电压 */
    volatile uint16_t      r_temper;    /* 读温度 */
    volatile uint16_t      crc;         /* crc校验位 */
}rs485_typedef;


/* RS485 本从机的地址  */
#define   bldc_addr                  0xf1
/* RS485 命令 定义 */
#define command_read                   0xf3
#define command_write                  0xf6


/******************************************************************************************/

/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */
#define RS485_RE(x)   do{ x ? \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
                      }while(0)





void rs485_init( uint32_t baudrate);  /* RS485初始化 */

unsigned short ModbusCRC16_Check(uint8_t *array, int BitNumber);

void ESP32_fetinst(uint8_t mode);

#endif


