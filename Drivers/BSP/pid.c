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

#include "./BSP/pid.h"
#include "./BSP/adc.h"
#include "./BSP/bldc.h"

uint8_t  pid_sta = 1;

PID_TypeDef  g_MA_speed_pid = { 0 };           /* 速度环PID参数结构体 */
PID_TypeDef  g_MA_current_pid = { 0 };         /* 电流环PID参数结构体 */

PID_TypeDef  g_MB_speed_pid = { 0 };           /* 速度环PID参数结构体 */
PID_TypeDef  g_MB_current_pid = { 0 };         /* 电流环PID参数结构体 */

TIM_HandleTypeDef g_tim2_handle;

/**
 * @brief       定时计算PID,在定时器更新中断里计算出pid期望值
 * @param       计算的频率,取值范围10-1000
 * @retval      无
 */
static void pid_timer_init(uint16_t hz )
{
    while( hz > 1000 && hz < 10);
        
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    g_tim2_handle.Instance = TIM2;
    g_tim2_handle.Init.Prescaler = 720-1;                        /* 定时器分频 */
    g_tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 计数模式 */
    g_tim2_handle.Init.Period = 100000 / hz - 1;                           /* 自动重装载值 */
    g_tim2_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR分频因子 */
    g_tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_tim2_handle.Init.RepetitionCounter = 0;                  /* 重复计数*/
    HAL_TIM_Base_Init(&g_tim2_handle);
    /* 优先级不用太高 */
    HAL_NVIC_SetPriority(TIM2_IRQn,8,0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&g_tim2_handle);
    
}

/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
void pid_init(uint16_t hz )
{
    /***************MATOR A********************/
    
    /* 速度环PID参数初始化 */
    g_MA_speed_pid.SetPoint = 0;       /* 设定目标值 */
    g_MA_speed_pid.ActualValue = 0.0;  /* 期望输出值 */
    g_MA_speed_pid.SumError = 0.0;     /* 积分值 */
    g_MA_speed_pid.Error = 0.0;        /* Error[1] */
    g_MA_speed_pid.LastError = 0.0;    /* Error[-1] */
    g_MA_speed_pid.PrevError = 0.0;    /* Error[-2] */
    g_MA_speed_pid.Proportion = KP_S;    /* 比例常数 Proportional Const */
    g_MA_speed_pid.Integral = KI_S;      /* 积分常数 Integral Const */
    g_MA_speed_pid.Derivative = KD_S;    /* 微分常数 Derivative Const */ 
    
    /* 电流环PID参数初始化 */
    g_MA_current_pid.SetPoint = 0;       /* 设定目标值 */
    g_MA_current_pid.ActualValue = 0.0;  /* 期望输出值 */
    g_MA_current_pid.SumError = 0.0;     /* 积分值 */
    g_MA_current_pid.Error = 0.0;        /* Error[1] */
    g_MA_current_pid.LastError = 0.0;    /* Error[-1] */
    g_MA_current_pid.PrevError = 0.0;    /* Error[-2] */
    g_MA_current_pid.Proportion = KP_C;    /* 比例常数 Proportional Const */
    g_MA_current_pid.Integral = KI_C;      /* 积分常数 Integral Const */
    g_MA_current_pid.Derivative = KD_C;    /* 微分常数 Derivative Const */ 
    
    /***************MATOR B********************/
    
    /* 速度环PID参数初始化 */
    g_MB_speed_pid.SetPoint = 0;       /* 设定目标值 */
    g_MB_speed_pid.ActualValue = 0.0;  /* 期望输出值 */
    g_MB_speed_pid.SumError = 0.0;     /* 积分值 */
    g_MB_speed_pid.Error = 0.0;        /* Error[1] */
    g_MB_speed_pid.LastError = 0.0;    /* Error[-1] */
    g_MB_speed_pid.PrevError = 0.0;    /* Error[-2] */
    g_MB_speed_pid.Proportion = KP_S;    /* 比例常数 Proportional Const */
    g_MB_speed_pid.Integral = KI_S;      /* 积分常数 Integral Const */
    g_MB_speed_pid.Derivative = KD_S;    /* 微分常数 Derivative Const */ 
    
    /* 电流环PID参数初始化 */
    g_MB_current_pid.SetPoint = 0;       /* 设定目标值 */
    g_MB_current_pid.ActualValue = 0.0;  /* 期望输出值 */
    g_MB_current_pid.SumError = 0.0;     /* 积分值 */
    g_MB_current_pid.Error = 0.0;        /* Error[1] */
    g_MB_current_pid.LastError = 0.0;    /* Error[-1] */
    g_MB_current_pid.PrevError = 0.0;    /* Error[-2] */
    g_MB_current_pid.Proportion = KP_C;    /* 比例常数 Proportional Const */
    g_MB_current_pid.Integral = KI_C;      /* 积分常数 Integral Const */
    g_MB_current_pid.Derivative = KD_C;    /* 微分常数 Derivative Const */ 
    
    
    //打开定时器
    pid_timer_init(hz);
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
    
#if  INCR_LOCT_SELECT                                                       /* 增量式PID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
    PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
    PID->LastError = PID->Error;
    
#else                                                                       /* 位置式PID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                       + (PID->Integral * PID->SumError)                    /* 积分环节 */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
    PID->LastError = PID->Error;
    
#endif
    
    /* 限制值在有效范围内 */
    if(PID->ActualValue > MAX_PWM)
    {
        PID->ActualValue = MAX_PWM;
    }
    else if(PID->ActualValue < 0)
    {
        PID->ActualValue = 0;
    }
    
    return ((uint32_t)(PID->ActualValue));                                   /* 返回计算后输出的数值 */
}


/**
 * @brief       中断服务函数，不调用公共处理函数。
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
                
                /* 实际值为0时限制 */
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

                /* 实际值为0时限制 */
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
