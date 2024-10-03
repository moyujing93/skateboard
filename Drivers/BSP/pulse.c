/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID�㷨����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com/forum.php
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */
#include <stdlib.h>
#include "./BSP/pulse.h"
#include "./BSP/bldc.h"
#include "./BSP/pid.h"


volatile uint16_t g_timer_fq = 0;
volatile uint16_t g_timer_duty = 0;
volatile uint16_t fq_ccr,duty_ccr;
TIM_HandleTypeDef     g_tim4_handle;
TIM_IC_InitTypeDef   g_tim4_iccof_handle;

/**
 * @brief       ��ʼ�����벶��ʱ��
 * @param       ��Χ20HZ-1000HZ
 * @retval      ��
 */
void pulse_init(void)
{
    GPIO_InitTypeDef     ic_init_hander;
    
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    ic_init_hander.Mode = GPIO_MODE_AF_INPUT;
    ic_init_hander.Pin = GPIO_PIN_8;
    ic_init_hander.Pull = GPIO_PULLUP;
    ic_init_hander.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&ic_init_hander);
    
    
    g_tim4_handle.Instance = TIM4;
    g_tim4_handle.Init.Prescaler = 72-1;                        /* ��ʱ����Ƶ */
    g_tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ����ģʽ */
    g_tim4_handle.Init.Period = 50000-1;                         /* �Զ���װ��ֵ */
    g_tim4_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR��Ƶ���� */
    g_tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_tim4_handle.Init.RepetitionCounter = 0;                  /* �ظ�����*/
    HAL_TIM_Base_Start_IT(&g_tim4_handle);
    HAL_TIM_IC_Init(&g_tim4_handle);
    
    g_tim4_iccof_handle.ICFilter = 0x2;
    g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_RISING;
    g_tim4_iccof_handle.ICPrescaler = TIM_ICPSC_DIV1;
    g_tim4_iccof_handle.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&g_tim4_handle, &g_tim4_iccof_handle, TIM_CHANNEL_3);
    HAL_NVIC_SetPriority(TIM4_IRQn,8,0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_IC_Start_IT(&g_tim4_handle,TIM_CHANNEL_3);
    
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_tim4_handle, TIM_FLAG_CC3) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&g_tim4_handle, TIM_IT_CC3);
        
        /* Input capture event */
        if (g_tim4_iccof_handle.ICPolarity == TIM_ICPOLARITY_RISING)
        {
            //�ı䲶׽����
            TIM4->CCER |= TIM_CCER_CC3P;
            g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_FALLING;
            
            //��������ʱ��
            fq_ccr = HAL_TIM_ReadCapturedValue(&g_tim4_handle,TIM_CHANNEL_3) + 1;
            //����Ƶ��
            g_timer_fq = 1e6 / fq_ccr;
            //����ߵ�ƽʱ��
            g_timer_duty = duty_ccr;
            //����
            __HAL_TIM_SET_COUNTER(&g_tim4_handle,0);
        }else
        {
            //�ı䲶׽����
            TIM4->CCER &= ~TIM_CCER_CC3P;
            g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_RISING;
            //����ߵ�ƽʱ��
            duty_ccr = HAL_TIM_ReadCapturedValue(&g_tim4_handle,TIM_CHANNEL_3) + 1;
        }
    }else if (__HAL_TIM_GET_FLAG(&g_tim4_handle, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&g_tim4_handle, TIM_IT_UPDATE);
        g_timer_fq = 0;
        g_timer_duty = 0;
        duty_ccr = 0;
        fq_ccr = 0;
        //��Ϊ��׽���������أ���ֹͻȻ���ź�ʱ���պ����½��ز�׽�������ͻ��д���ĸߵ�ƽʱ��
        TIM4->CCER &= ~TIM_CCER_CC3P;
        g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_RISING;
        
    }
}


/**
 * @brief       RC���Ƶ��
 * @param       1=ת�ٿ��� 0 = ��������
 * @retval      
 */
void pulse_fetinst(uint8_t mode)
{
    static int      motor_control;
    
    //�ױ�ȫ�ֱ�����ò�Ҫֱ��ʹ��
    uint16_t timer_fq = g_timer_fq;
    uint16_t timer_duty = g_timer_duty;
    
    //ֻ��������Ϊ20-200hz��pwm
    if(timer_fq > 20 && timer_fq < 200)
    {
        //ֻ���ܸߵ�ƽ1000us-2000us��pwm
        if(timer_duty >= 1000 && timer_duty <= 2000)
        {
            motor_control = timer_duty - 1500;
            
            if(mode)      //ת�ٿ���
            {
                g_MA_current_pid.SetPoint  = MAX_CURRENT / 2;
                g_MB_current_pid.SetPoint  = MAX_CURRENT / 2;
                
                //��ת����ת��ֹͣ
                if(motor_control < -50)
                {
                    g_bldc_motorA.dir = CCW;
                    g_bldc_motorA.run_flag = RUN;
                    
                    g_bldc_motorB.dir = CW;
                    g_bldc_motorB.run_flag = RUN;
                    
                    g_MA_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
                    g_MB_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
                }else if(motor_control > 50)
                {
                    g_bldc_motorA.dir = CW;
                    g_bldc_motorA.run_flag = RUN;
                    
                    g_bldc_motorB.dir = CCW;
                    g_bldc_motorB.run_flag = RUN;
                    
                    g_MA_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
                    g_MB_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
                    
                }else
                {
                    g_bldc_motorA.run_flag = STOP;
                    g_bldc_motorB.run_flag = STOP;
                    
                    g_MA_speed_pid.SetPoint  = 0;
                    g_MA_current_pid.SetPoint  = 0;
                    
                    g_MB_speed_pid.SetPoint  = 0;
                    g_MB_current_pid.SetPoint  = 0;
                }
                
            }else    //��������
            {
                g_MA_speed_pid.SetPoint  = MAX_RPM;
                g_MB_speed_pid.SetPoint  = MAX_RPM;
                
                //��ת����ת��ֹͣ
                if(motor_control > 50)
                {
                    g_bldc_motorA.dir = CCW;
                    g_bldc_motorA.run_flag = RUN;
                    
                    g_bldc_motorB.dir = CW;
                    g_bldc_motorB.run_flag = RUN;
                    
                    g_MA_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
                    g_MB_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
                }else if(motor_control < -50)
                {
                    g_bldc_motorA.dir = CW;
                    g_bldc_motorA.run_flag = RUN;
                    
                    g_bldc_motorB.dir = CCW;
                    g_bldc_motorB.run_flag = RUN;
                    
                    g_MA_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
                    g_MB_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
                }else
                {
                    g_bldc_motorA.run_flag = STOP;
                    g_bldc_motorB.run_flag = STOP;
                    
                    g_MA_speed_pid.SetPoint  = 0;
                    g_MA_current_pid.SetPoint  = 0;
                    
                    g_MB_speed_pid.SetPoint  = 0;
                    g_MB_current_pid.SetPoint  = 0;
                }
            }
            
        }
        
    }else
    {
        g_bldc_motorA.run_flag = STOP;
        g_bldc_motorB.run_flag = STOP;
        g_MA_current_pid.SetPoint  = 0;
        g_MA_speed_pid.SetPoint  = 0;
        g_MB_speed_pid.SetPoint  = 0;
        g_MB_current_pid.SetPoint  = 0;
    }
}



