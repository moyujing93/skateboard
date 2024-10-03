/**
 ****************************************************************************************************
 * @file        spi.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       SPI ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 */

#ifndef _ADC_H
#define _ADC_H
#include "./SYSTEM/sys/sys.h"
/***************************************** �ṹ��ͱ��� *************************************************/

#define MB_CUN_PIN             GPIO_PIN_0
#define MB_CUN_PORT            GPIOA

#define MA_CUN_PIN             GPIO_PIN_1  //�°��ض���
#define MA_CUN_PORT            GPIOA

#define V_B_PIN                GPIO_PIN_4
#define V_B_PORT               GPIOA
   
#define V_T_PIN                GPIO_PIN_5
#define V_T_PORT               GPIOA


/***********************************************************************************************/


typedef struct
{
    volatile uint16_t  MB_cun;  /* ���B���� */  //�°��ض���
    volatile uint16_t  MA_cun;  /* ���A���� */
    volatile uint16_t  V_B;    /* ���ߵ�ѹ */
    volatile uint16_t  V_T;    /* �������¶� */
}GET_ADC_typedef;
/*  �ṹ���������������DMA��������  */
#define DATA_NUM               4

/***************************************** ���� *************************************************/
void adc1_dma_init(void);
void adc_dma_conver(uint8_t num);




















#endif
