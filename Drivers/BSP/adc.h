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

#define MA_CUN_PORT            GPIOA
#define MA_CUN_U_PIN           GPIO_PIN_0
#define MA_CUN_V_PIN           GPIO_PIN_1
#define MA_CUN_W_PIN           GPIO_PIN_2

#define MB_CUN_PORT            GPIOA
#define MB_CUN_U_PIN           GPIO_PIN_3
#define MB_CUN_V_PIN           GPIO_PIN_4
#define MB_CUN_W_PIN           GPIO_PIN_5

#define V_B_PIN                GPIO_PIN_4
#define V_B_PORT               GPIOC
   
#define V_T_PIN                GPIO_PIN_5
#define V_T_PORT               GPIOC


/***********************************************************************************************/


typedef struct
{
    volatile uint16_t  MA_U_cun;  /* 电机A电流 */
    volatile uint16_t  MA_V_cun;  /* 电机A电流 */
    volatile uint16_t  MA_W_cun;  /* 电机A电流 */
    volatile uint16_t  MB_U_cun;  /* 电机B电流 */
    volatile uint16_t  MB_V_cun;  /* 电机B电流 */
    volatile uint16_t  MB_W_cun;  /* 电机B电流 */
    volatile uint16_t  V_B;    /* 主线电压 */
    volatile uint16_t  V_T;    /* 驱动板温度 */
}GET_ADC_typedef;
/*  结构体的数量用于设置DMA传输数量  */
#define DATA_NUM               8

/***************************************** 函数 *************************************************/
void adc1_dma_init(void);
void adc_dma_conver(uint8_t num);




















#endif
