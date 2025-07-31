#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/

#define UART_TX_PORT                  GPIOB
#define UART_TX_PIN                   GPIO_PIN_6

#define UART_RX_PORT                  GPIOB
#define UART_RX_PIN                   GPIO_PIN_7


/******************************************************************************************/


// 定义 ESP32 NOW 的 信息结构体
typedef struct {
  //发送的信息
  uint8_t  sta;
  uint16_t adc;      /* 油门 */
  uint8_t mode;      /* 电流:转速 */
  uint8_t dir;      /* 电机旋转方向 */
  uint8_t gears;      /* 档位 */
  uint16_t crc;      /* crc */
  uint8_t bb;
} struct_read;


// 定义 ESP32 NOW 的 信息结构体
typedef struct
{
  // 发送的信息
  uint8_t ff;
  uint16_t speed;   /* 油门 `*/
  uint16_t current; /* 电流:转速 */
  uint16_t v_bus;   /* 电机旋转方向 */
  uint16_t v_tee;   /* 档位 */
  uint16_t crc;      /* crc */
  uint8_t bb;
} struct_uart_send;

//extern UART_HandleTypeDef g_uart1_handle;       /* HAL UART句柄 */
extern volatile uint8_t g_esp32_sta;
extern volatile struct_read   g_esp32_struct;

void MX_USART1_UART_Init(void);
uint8_t usart_receive_data(void);
void usart_send_data(uint8_t *buf, uint8_t len);

#endif


