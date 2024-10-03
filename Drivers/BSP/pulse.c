/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com/forum.php
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
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
 * @brief       初始化输入捕获定时器
 * @param       范围20HZ-1000HZ
 * @retval      无
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
    g_tim4_handle.Init.Prescaler = 72-1;                        /* 定时器分频 */
    g_tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 计数模式 */
    g_tim4_handle.Init.Period = 50000-1;                         /* 自动重装载值 */
    g_tim4_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR分频因子 */
    g_tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_tim4_handle.Init.RepetitionCounter = 0;                  /* 重复计数*/
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
            //改变捕捉方向
            TIM4->CCER |= TIM_CCER_CC3P;
            g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_FALLING;
            
            //计算脉宽时间
            fq_ccr = HAL_TIM_ReadCapturedValue(&g_tim4_handle,TIM_CHANNEL_3) + 1;
            //计算频率
            g_timer_fq = 1e6 / fq_ccr;
            //计算高电平时间
            g_timer_duty = duty_ccr;
            //清零
            __HAL_TIM_SET_COUNTER(&g_tim4_handle,0);
        }else
        {
            //改变捕捉方向
            TIM4->CCER &= ~TIM_CCER_CC3P;
            g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_RISING;
            //计算高电平时间
            duty_ccr = HAL_TIM_ReadCapturedValue(&g_tim4_handle,TIM_CHANNEL_3) + 1;
        }
    }else if (__HAL_TIM_GET_FLAG(&g_tim4_handle, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&g_tim4_handle, TIM_IT_UPDATE);
        g_timer_fq = 0;
        g_timer_duty = 0;
        duty_ccr = 0;
        fq_ccr = 0;
        //改为捕捉方向上升沿，防止突然有信号时，刚好是下降沿捕捉，这样就会有错误的高电平时间
        TIM4->CCER &= ~TIM_CCER_CC3P;
        g_tim4_iccof_handle.ICPolarity = TIM_ICPOLARITY_RISING;
        
    }
}


/**
 * @brief       RC控制电机
 * @param       1=转速控制 0 = 电流控制
 * @retval      
 */
void pulse_fetinst(uint8_t mode)
{
    static int      motor_control;
    
    //易变全局变量最好不要直接使用
    uint16_t timer_fq = g_timer_fq;
    uint16_t timer_duty = g_timer_duty;
    
    //只接受脉宽为20-200hz的pwm
    if(timer_fq > 20 && timer_fq < 200)
    {
        //只接受高电平1000us-2000us的pwm
        if(timer_duty >= 1000 && timer_duty <= 2000)
        {
            motor_control = timer_duty - 1500;
            
            if(mode)      //转速控制
            {
                g_MA_current_pid.SetPoint  = MAX_CURRENT / 2;
                g_MB_current_pid.SetPoint  = MAX_CURRENT / 2;
                
                //正转，反转，停止
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
                
            }else    //电流控制
            {
                g_MA_speed_pid.SetPoint  = MAX_RPM;
                g_MB_speed_pid.SetPoint  = MAX_RPM;
                
                //正转，反转，停止
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



