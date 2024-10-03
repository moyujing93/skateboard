/**
 ****************************************************************************************************
 * @file        spi.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       SPI 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef _ADC_H
#define _ADC_H
#include "./SYSTEM/sys/sys.h"
/***************************************** 结构体和变量 *************************************************/

#define MB_CUN_PIN             GPIO_PIN_0
#define MB_CUN_PORT            GPIOA

#define MA_CUN_PIN             GPIO_PIN_1  //新板重定义
#define MA_CUN_PORT            GPIOA

#define V_B_PIN                GPIO_PIN_4
#define V_B_PORT               GPIOA
   
#define V_T_PIN                GPIO_PIN_5
#define V_T_PORT               GPIOA


/***********************************************************************************************/


typedef struct
{
    volatile uint16_t  MB_cun;  /* 电机B电流 */  //新板重定义
    volatile uint16_t  MA_cun;  /* 电机A电流 */
    volatile uint16_t  V_B;    /* 主线电压 */
    volatile uint16_t  V_T;    /* 驱动板温度 */
}GET_ADC_typedef;
/*  结构体的数量用于设置DMA传输数量  */
#define DATA_NUM               4

/***************************************** 函数 *************************************************/
void adc1_dma_init(void);
void adc_dma_conver(uint8_t num);




















#endif
