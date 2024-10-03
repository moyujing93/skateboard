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

#include "./BSP/pid.h"
#include "./BSP/adc.h"
#include "./BSP/bldc.h"

uint8_t  pid_sta = 1;

PID_TypeDef  g_MA_speed_pid = { 0 };           /* �ٶȻ�PID�����ṹ�� */
PID_TypeDef  g_MA_current_pid = { 0 };         /* ������PID�����ṹ�� */

PID_TypeDef  g_MB_speed_pid = { 0 };           /* �ٶȻ�PID�����ṹ�� */
PID_TypeDef  g_MB_current_pid = { 0 };         /* ������PID�����ṹ�� */

TIM_HandleTypeDef g_tim2_handle;

/**
 * @brief       ��ʱ����PID,�ڶ�ʱ�������ж�������pid����ֵ
 * @param       �����Ƶ��,ȡֵ��Χ10-1000
 * @retval      ��
 */
static void pid_timer_init(uint16_t hz )
{
    while( hz > 1000 && hz < 10);
        
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    g_tim2_handle.Instance = TIM2;
    g_tim2_handle.Init.Prescaler = 720-1;                        /* ��ʱ����Ƶ */
    g_tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ����ģʽ */
    g_tim2_handle.Init.Period = 100000 / hz - 1;                           /* �Զ���װ��ֵ */
    g_tim2_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR��Ƶ���� */
    g_tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_tim2_handle.Init.RepetitionCounter = 0;                  /* �ظ�����*/
    HAL_TIM_Base_Init(&g_tim2_handle);
    /* ���ȼ�����̫�� */
    HAL_NVIC_SetPriority(TIM2_IRQn,8,0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&g_tim2_handle);
    
}

/**
 * @brief       pid��ʼ��
 * @param       ��
 * @retval      ��
 */
void pid_init(uint16_t hz )
{
    /***************MATOR A********************/
    
    /* �ٶȻ�PID������ʼ�� */
    g_MA_speed_pid.SetPoint = 0;       /* �趨Ŀ��ֵ */
    g_MA_speed_pid.ActualValue = 0.0;  /* �������ֵ */
    g_MA_speed_pid.SumError = 0.0;     /* ����ֵ */
    g_MA_speed_pid.Error = 0.0;        /* Error[1] */
    g_MA_speed_pid.LastError = 0.0;    /* Error[-1] */
    g_MA_speed_pid.PrevError = 0.0;    /* Error[-2] */
    g_MA_speed_pid.Proportion = KP_S;    /* �������� Proportional Const */
    g_MA_speed_pid.Integral = KI_S;      /* ���ֳ��� Integral Const */
    g_MA_speed_pid.Derivative = KD_S;    /* ΢�ֳ��� Derivative Const */ 
    
    /* ������PID������ʼ�� */
    g_MA_current_pid.SetPoint = 0;       /* �趨Ŀ��ֵ */
    g_MA_current_pid.ActualValue = 0.0;  /* �������ֵ */
    g_MA_current_pid.SumError = 0.0;     /* ����ֵ */
    g_MA_current_pid.Error = 0.0;        /* Error[1] */
    g_MA_current_pid.LastError = 0.0;    /* Error[-1] */
    g_MA_current_pid.PrevError = 0.0;    /* Error[-2] */
    g_MA_current_pid.Proportion = KP_C;    /* �������� Proportional Const */
    g_MA_current_pid.Integral = KI_C;      /* ���ֳ��� Integral Const */
    g_MA_current_pid.Derivative = KD_C;    /* ΢�ֳ��� Derivative Const */ 
    
    /***************MATOR B********************/
    
    /* �ٶȻ�PID������ʼ�� */
    g_MB_speed_pid.SetPoint = 0;       /* �趨Ŀ��ֵ */
    g_MB_speed_pid.ActualValue = 0.0;  /* �������ֵ */
    g_MB_speed_pid.SumError = 0.0;     /* ����ֵ */
    g_MB_speed_pid.Error = 0.0;        /* Error[1] */
    g_MB_speed_pid.LastError = 0.0;    /* Error[-1] */
    g_MB_speed_pid.PrevError = 0.0;    /* Error[-2] */
    g_MB_speed_pid.Proportion = KP_S;    /* �������� Proportional Const */
    g_MB_speed_pid.Integral = KI_S;      /* ���ֳ��� Integral Const */
    g_MB_speed_pid.Derivative = KD_S;    /* ΢�ֳ��� Derivative Const */ 
    
    /* ������PID������ʼ�� */
    g_MB_current_pid.SetPoint = 0;       /* �趨Ŀ��ֵ */
    g_MB_current_pid.ActualValue = 0.0;  /* �������ֵ */
    g_MB_current_pid.SumError = 0.0;     /* ����ֵ */
    g_MB_current_pid.Error = 0.0;        /* Error[1] */
    g_MB_current_pid.LastError = 0.0;    /* Error[-1] */
    g_MB_current_pid.PrevError = 0.0;    /* Error[-2] */
    g_MB_current_pid.Proportion = KP_C;    /* �������� Proportional Const */
    g_MB_current_pid.Integral = KI_C;      /* ���ֳ��� Integral Const */
    g_MB_current_pid.Derivative = KD_C;    /* ΢�ֳ��� Derivative Const */ 
    
    
    //�򿪶�ʱ��
    pid_timer_init(hz);
}

/**
 * @brief       pid�ջ�����
 * @param       *PID��PID�ṹ�������ַ
 * @param       Feedback_value����ǰʵ��ֵ
 * @retval      �������ֵ
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* ����ƫ�� */
    
#if  INCR_LOCT_SELECT                                                       /* ����ʽPID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* �������� */
                        + (PID->Integral * PID->Error)                                             /* ���ֻ��� */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* ΢�ֻ��� */
    
    PID->PrevError = PID->LastError;                                        /* �洢ƫ������´μ��� */
    PID->LastError = PID->Error;
    
#else                                                                       /* λ��ʽPID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* �������� */
                       + (PID->Integral * PID->SumError)                    /* ���ֻ��� */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* ΢�ֻ��� */
    PID->LastError = PID->Error;
    
#endif
    
    /* ����ֵ����Ч��Χ�� */
    if(PID->ActualValue > MAX_PWM)
    {
        PID->ActualValue = MAX_PWM;
    }
    else if(PID->ActualValue < 0)
    {
        PID->ActualValue = 0;
    }
    
    return ((uint32_t)(PID->ActualValue));                                   /* ���ؼ�����������ֵ */
}


/**
 * @brief       �жϷ������������ù�����������
 * @param       
 * @retval      
 */
void TIM2_IRQHandler(void)
{
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&g_tim2_handle, TIM_FLAG_UPDATE) != RESET)
    {
        static uint16_t pwm_temp1;
        static uint16_t pwm_temp2;
        
        
        __HAL_TIM_CLEAR_IT(&g_tim2_handle, TIM_IT_UPDATE);
        
        
        
        if(pid_sta)
        {
            /***************MATOR A********************/
            
            if(g_bldc_motorA.run_flag == RUN  &&  g_bldc_motorA.max_t == RESET  && g_bldc_motorA.locked_rotor == RESET)
            {
                if (g_bldc_motorA.max_c == SET)
                {
    //                g_MA_current_pid.SetPoint = g_MA_current_pid.SetPoint / 2;
                }
                pwm_temp1 = increment_pid_ctrl(&g_MA_current_pid,g_bldc_motorA.current);
                pwm_temp2 = increment_pid_ctrl(&g_MA_speed_pid,g_bldc_motorA.speed);
                
                /* ʵ��ֵΪ0ʱ���� */
                if(g_bldc_motorA.speed < 50)
                {
                    if(g_MA_speed_pid.ActualValue > 50)
                    {
                        g_MA_speed_pid.ActualValue = 50;
                        pwm_temp2 = 50;
                    }
                }

                if(pwm_temp1 >= pwm_temp2)
                {
                  g_bldc_motorA.pwm_duty = pwm_temp2;
                  g_MA_current_pid.ActualValue = g_MA_speed_pid.ActualValue;
                }
                else
                {
                  g_bldc_motorA.pwm_duty = pwm_temp1;
                  g_MA_speed_pid.ActualValue = g_MA_current_pid.ActualValue;
                }
            }else
            {
                g_MA_speed_pid.ActualValue = 0;
                g_MA_speed_pid.LastError = 0;
                g_MA_speed_pid.PrevError = 0;
                g_MA_current_pid.ActualValue = 0;
                g_MA_current_pid.LastError = 0;
                g_MA_current_pid.PrevError = 0;
                g_bldc_motorA.pwm_duty = 0;
            }
            
            /***************MATOR B********************/
            
            if(g_bldc_motorB.run_flag == RUN  &&  g_bldc_motorB.max_t == RESET  && g_bldc_motorB.locked_rotor == RESET)
            {
                if (g_bldc_motorB.max_c == SET)
                {
    //                g_MA_current_pid.SetPoint = g_MA_current_pid.SetPoint / 2;
                }
                pwm_temp1 = increment_pid_ctrl(&g_MB_current_pid,g_bldc_motorB.current);
                pwm_temp2 = increment_pid_ctrl(&g_MB_speed_pid,g_bldc_motorB.speed);

                /* ʵ��ֵΪ0ʱ���� */
                if(g_bldc_motorB.speed < 100)
                {
                    if(g_MB_speed_pid.ActualValue > 200 + ((g_bldc_motorB.speed / 100.0f) * (MAX_PWM - 200)))
                    {
                        g_MB_speed_pid.ActualValue = 200 + ((g_bldc_motorB.speed / 100.0f) * (MAX_PWM - 200));
                        pwm_temp2 = g_MB_speed_pid.ActualValue;
                    }
                }

                if(pwm_temp1 >= pwm_temp2)
                {
                  g_bldc_motorB.pwm_duty = pwm_temp2;
                  g_MB_current_pid.ActualValue = g_MB_speed_pid.ActualValue;
                }
                else
                {
                  g_bldc_motorB.pwm_duty = pwm_temp1;
                  g_MB_speed_pid.ActualValue = g_MB_current_pid.ActualValue;
                }
            }else
            {
                g_MB_speed_pid.ActualValue = 0;
                g_MB_speed_pid.LastError = 0;
                g_MB_speed_pid.PrevError = 0;
                g_MB_current_pid.ActualValue = 0;
                g_MB_current_pid.LastError = 0;
                g_MB_current_pid.PrevError = 0;
                g_bldc_motorB.pwm_duty = 0;
            }
            
            
        }
    }
}
