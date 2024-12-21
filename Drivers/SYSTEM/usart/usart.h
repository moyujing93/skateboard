/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       串口初始化代码(一般是串口1)，支持printf
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
 * V1.0 20211103
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 和 串口 定义 
 * USART1重映射AFIO到GPIOB6,7
 * 
 */
#define USART_TX_GPIO_PORT                  GPIOB
#define USART_TX_GPIO_PIN                   GPIO_PIN_6

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_7


/******************************************************************************************/

#define USART_REC_LEN               250         /* 定义最大接收字节数 200 */
#define USART_EN_RX                 1           /* 使能（1）/禁止（0）串口1接收 */


// 定义 ESP32 NOW 的 信息结构体
typedef struct {
  //发送的信息
  uint8_t  sta;
  uint16_t adc;      /* 油门 */
  uint8_t mode;      /* 电流:转速 */
  uint8_t dir;      /* 电机旋转方向 */
  uint8_t gears;      /* 档位 */
  uint8_t bb;
} struct_read;


// 定义 ESP32 NOW 的 信息结构体
typedef struct
{
  // 发送的信息
  uint8_t ff;
  uint16_t speed;   /* 油门 */
  uint16_t current; /* 电流:转速 */
  uint16_t v_bus;   /* 电机旋转方向 */
  uint16_t v_tee;   /* 档位 */
  uint8_t bb;
} struct_uart_send;

//extern UART_HandleTypeDef g_uart1_handle;       /* HAL UART句柄 */
extern volatile uint8_t g_usart_rx_sta;
/*  接收到多少字节 */
extern volatile uint8_t g_usart_rx_num;

extern volatile struct_read   g_esp32_struct;

void usart_init(uint32_t bound);                /* 串口初始化函数 */
void usart_send_data(uint8_t *buf, uint8_t len);
uint8_t usart_receive_data(uint8_t *buf, uint16_t buf_size);

#endif


