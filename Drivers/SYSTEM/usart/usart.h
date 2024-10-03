/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211103
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ���� �� ���� ���� 
 * USART1��ӳ��AFIO��GPIOB6,7
 * 
 */
#define USART_TX_GPIO_PORT                  GPIOB
#define USART_TX_GPIO_PIN                   GPIO_PIN_6
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_7
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define USART_UX                            USART1
#define USART_UX_IRQn                       USART1_IRQn
#define USART_UX_IRQHandler                 USART1_IRQHandler
#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_USART1_CLK_ENABLE(); }while(0)

/******************************************************************************************/

#define USART_REC_LEN               250         /* �����������ֽ��� 200 */
#define USART_EN_RX                 1           /* ʹ�ܣ�1��/��ֹ��0������1���� */
#define RXBUFFERSIZE                1           /* �����С */


// ���� ESP32 NOW �� ��Ϣ�ṹ��
typedef struct {
  //���͵���Ϣ
  uint8_t  sta;
  uint16_t adc;      /* ���� */
  uint8_t mode;      /* ����:ת�� */
  uint8_t dir;      /* �����ת���� */
  uint8_t gears;      /* ��λ */
  uint8_t bb;
} struct_send;


//extern UART_HandleTypeDef g_uart1_handle;       /* HAL UART��� */
extern volatile uint8_t g_usart_rx_sta;
/*  ���յ������ֽ� */
extern volatile uint8_t g_usart_rx_num;

extern volatile struct_send   g_esp32_struct;

extern volatile uint32_t esp32_lose_time;

void usart_init(uint32_t bound);                /* ���ڳ�ʼ������ */
void usart_send_data(uint8_t *buf, uint8_t len);
uint8_t usart_receive_data(uint8_t *buf, uint16_t buf_size);

#endif


