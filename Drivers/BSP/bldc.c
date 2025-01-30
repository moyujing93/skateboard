/**
 ****************************************************************************************************
 * @file        bldc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/bldc.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/bldc_six_step.h"



uint8_t  G_DeadTime    = 0x1E;          //死区时间

uint16_t MAX_PWM        =  MAX_PWM_SET;       //最大占空比0-1000 
uint16_t MAX_PWM_BRAKE  =  MAX_PWM_BRAKE_SET;       //最大占空比0-1000 


//100,110,010,011,001,101
/*  函数指针数组 */
pctr pfunclist_motorA_cw[6] =
{
    &MA_whvl,&MA_vhul,&MA_whul,
    &MA_uhwl,&MA_uhvl,&MA_vhwl
    
};

pctr pfunclist_motorA_ccw[6] =
{
    &MA_vhwl,&MA_uhvl,&MA_uhwl,
    &MA_whul,&MA_vhul,&MA_whvl
};
pctr pfunclist_motorA_bk[4] =
{
    &MA_br_uhvwl,&MA_br_whuvl,&MA_br_vhuwl,&MA_br_LLL
};

pctr pfunclist_motorB_cw[6] =
{
    &MB_whvl,&MB_vhul,&MB_whul,
    &MB_uhwl,&MB_uhvl,&MB_vhwl
};

pctr pfunclist_motorB_ccw[6] =
{
    &MB_vhwl,&MB_uhvl,&MB_uhwl,
    &MB_whul,&MB_vhul,&MB_whvl
};

pctr pfunclist_motorB_bk[4] =
{
    &MB_br_uhvwl,&MB_br_whuvl,&MB_br_vhuwl,&MB_br_LLL
};

 
/* 结构体初始值 */
_bldc_obj    g_bldc_motorA = {0};
_bldc_obj    g_bldc_motorB = {0};
_gtime_obj   g_bldc_time  =  {0};

/* HAL库所需结构体 */
TIM_HandleTypeDef   g_MA_timx_handle;
TIM_HandleTypeDef   g_MB_timx_handle;


/**
 * @brief       整形限幅函数
 * @param       变量
 * @param       最小值
 * @param       最大值
 * @retval      
 */
int int_limit(int num,int min ,int max)
{
    int get_num = num;
    if(num > max)
    {
        get_num = max;
    }else if(num < min)
    {
        get_num = min;
    }
    return get_num;
}
/**
 * @brief       整形限幅函数
 * @param       变量
 * @param       最小值
 * @param       最大值
 * @retval      
 */
int int_abs(int num)
{
    if(num < 0)
    {
        return -num;
    }
    return num;
}


/***************************************** 函数部分 *************************************************/
/**
 * @brief       无刷电机初始化，包括定时器，霍尔接口引脚初始化
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void bldc_init(uint16_t arr, uint16_t psc)
{
    TIM_BreakDeadTimeConfigTypeDef g_sbreak_dead_time_config;
    TIM_OC_InitTypeDef  g_atimx_oc_chy_handle;
    GPIO_InitTypeDef gpio_init_struct;
    
    /**********************打开时钟************************/
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    
    
    /*********************MOTORA*************************/
    
    /* MOTORB上桥臂引脚初始化 */
    gpio_init_struct.Pin = MA_UH_PWM_PIN | MA_VH_PWM_PIN | MA_WH_PWM_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    
    /* 上桥臂TIM初始化 */
    g_MA_timx_handle.Instance = TIM8;
    g_MA_timx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_MA_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 计数模式 */
    g_MA_timx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_MA_timx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR分频因子 */
    g_MA_timx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_MA_timx_handle.Init.RepetitionCounter = 0;                  /* 重复计数*/
    HAL_TIM_PWM_Init(&g_MA_timx_handle);                          /* 初始化PWM */
    
    /* 配置输出比较模式 */
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择 */
    g_atimx_oc_chy_handle.Pulse = 0;                            /* 比较值 */
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 极性 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCPOLARITY_HIGH;   /* 互补通道极性 */
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    

    /* 设置死区参数，开启死区中断 */
    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    g_sbreak_dead_time_config.DeadTime = G_DeadTime;       /* 死区时间设置 */
    
    __HAL_TIM_MOE_ENABLE(&g_MA_timx_handle);  /* MOE=1,使能主输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MA_timx_handle, &g_sbreak_dead_time_config);
    

    /* 下桥臂引脚初始化 */
    gpio_init_struct.Pin = MA_VL_PIN | MA_WL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    gpio_init_struct.Pin = MA_UL_PIN;
    HAL_GPIO_Init(MA_UL_PORT, &gpio_init_struct);
    
    /* 霍尔引脚初始化 */
    gpio_init_struct.Pin = MA_HALL_U_PIN | MA_HALL_V_PIN | MA_HALL_W_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
    
    /*********************MOTORB*************************/
    
    /* MOTORB上桥臂引脚初始化 */
    gpio_init_struct.Pin = MB_UH_PWM_PIN | MB_VH_PWM_PIN | MB_WH_PWM_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    /* 上桥臂TIM初始化 */
    g_MB_timx_handle.Instance = TIM1;
    g_MB_timx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_MB_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 计数模式 */
    g_MB_timx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_MB_timx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR分频因子 */
    g_MB_timx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_MB_timx_handle.Init.RepetitionCounter = 0;                  /* 重复计数*/
    HAL_TIM_PWM_Init(&g_MB_timx_handle);                          /* 初始化PWM */
    
    /* 配置输出比较模式 */
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择 */
    g_atimx_oc_chy_handle.Pulse = 0;                            /* 比较值 */
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 极性 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCPOLARITY_HIGH;   /* 互补通道极性 */
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    
    /* 设置死区参数，开启死区中断 */
    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    g_sbreak_dead_time_config.DeadTime = G_DeadTime;       /* 死区时间设置 */
    
    __HAL_TIM_MOE_ENABLE(&g_MB_timx_handle);  /* MOE=1,使能主输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MB_timx_handle, &g_sbreak_dead_time_config);
    
    
    /* 下桥臂引脚初始化 */
    gpio_init_struct.Pin = MB_UL_PIN | MB_VL_PIN | MB_WL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
    /* 霍尔引脚初始化 */
    gpio_init_struct.Pin = MB_HALL_U_PIN | MB_HALL_V_PIN | MB_HALL_W_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    
    
    //电源引脚,有效信号高电平
    gpio_init_struct.Pin = MX_POWER_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MX_POWER_PORT, &gpio_init_struct);
    
    
    
    /******     注意打开顺序    *******/
    
    
    //打开TIM1的中断
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);               /* 优先级最高 */
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    
    
    HAL_TIM_Base_Start_IT(&g_MB_timx_handle);                 /* 启动时基单元的更新中断 */
    HAL_TIM_Base_Start(&g_MA_timx_handle);                 /* 启动时基单元的更新中断 */
    
    
    /* 开启定时器输出 */
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_3);
    
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_3);
    
    /* 开启互补通道 */
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_3);
    
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_3);
    
    
}


/**
 * @brief       获取霍尔传感器引脚状态
 * @param       
 * @retval      霍尔传感器引脚状态
 */
static uint8_t hallsensor_get_state(motornum_Type  motor_num)
{
    /*
            HU           HV         HW
            位2          位1        位0
    */
    uint8_t state = 0x00U;
    
    if(motor_num == MOTORA)
    {
        if(HAL_GPIO_ReadPin(MA_HALL_U_PORT,MA_HALL_U_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 2);
        }
        if(HAL_GPIO_ReadPin(MA_HALL_V_PORT,MA_HALL_V_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 1);
        }
        if(HAL_GPIO_ReadPin(MA_HALL_W_PORT,MA_HALL_W_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 0);
        }
    }else if(motor_num == MOTORB)
    {
        if(HAL_GPIO_ReadPin(MB_HALL_U_PORT,MB_HALL_U_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 2);
        }
        if(HAL_GPIO_ReadPin(MB_HALL_V_PORT,MB_HALL_V_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 1);
        }
        if(HAL_GPIO_ReadPin(MB_HALL_W_PORT,MB_HALL_W_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= (0x01 << 0);
        }
    }
    
    
    /* 60度霍尔使用 */
    #if 0
    if(state == 0x07U)
    {
        state = 0x02U;
    }else if(state == 0x00U)
    {
        state = 0x05U;
    }
    #endif
    
    
    return state;
}



/**
 * @brief       判断霍尔状态，计算1/2Ermp的时间
 * @param       
 * @retval      
 */
void hall_judge(_bldc_obj  *motor,uint8_t *hall_sta)
{
    uint8_t  hall_sta_temp = *hall_sta;
    //检测霍尔有没有异常
    if(motor->step_sta == 0 || motor->step_sta >= 7)
    {
        if(++motor->hall_erro_count > 20) motor->hall_miss = SET;
    }else
    {
        //记录霍尔出错到正常的时间，可以用来计算霍尔健康度
        motor->hall_erro_count = 0;
    }

    //电机速度，方向计算,测周法，优点响应非常快，测量范围广，缺点：处理噪波比较麻烦
    if(motor->step_count < 10000)  
    {
        motor->step_count++;
    }else
    {
        motor->step_all_time = motor->step_count;
    }
    
    if(motor->step_last != motor->step_sta)
    {
        //计算出1/2 Ermp的时间
        if(motor->step_sta == hall_sta_temp)
        {
            motor->step_all_time = motor->step_count;
            motor->step_count = 0;
            
            //改变判断方向，避免电机堵转抖动产生的噪波
            if(hall_sta_temp == 0x01)
            {
                hall_sta_temp = 0x06;
                if(motor->step_last == 0x03)
                {
                    motor->step_dir = CCW;
                }else if(motor->step_last == 0x05)
                {
                    motor->step_dir = CW;
                }
            }else if(hall_sta_temp == 0x06)
            {
                hall_sta_temp = 0x01;
                if(motor->step_last == 0x04)
                {
                    motor->step_dir = CCW;
                }else if(motor->step_last == 0x02)
                {
                    motor->step_dir = CW;
                }
            }
            
            *hall_sta = hall_sta_temp;
        }
        motor->step_last = motor->step_sta;
    }
}

/**
 * @brief       获取霍尔传感器引脚状态
 * @param       
 * @retval      霍尔传感器引脚状态
 */
static uint8_t brake_cun_set(_bldc_obj  *motor)
{
    uint8_t sta = 0;
    //低速用电流锁止
//    if(motor->speed < 100)
//    {
//        if(motor->step_sta == 1)
//        {
//            sta = 1;
//        }else if(motor->step_sta == 2)
//        {
//            sta = 2;
//        }else if(motor->step_sta == 4)
//        {
//            sta = 3;
//        }
//    }else if(motor->speed > 120)
//    {
//        //短接三相
//        sta = 4;
//    }
    if(motor->step_sta == 1)
    {
        sta = 1;
    }else if(motor->step_sta == 2)
    {
        sta = 2;
    }else if(motor->step_sta == 4)
    {
        sta = 3;
    }
    return sta;
}


/**
 * @brief       定时器中断服务函数
 * @param       无
 * @retval      无
 */
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_MB_timx_handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t   bk_mode = 0;    //刹车和运行状态切换标志
    static uint8_t  get_num = 0;     //局部计数
    static uint8_t  MA_step_ch = 1;    //测频以这个step为标准
    static uint8_t  MB_step_ch = 1;    //测频以这个step为标准
    
    if(htim->Instance == TIM1)
    {
        //获得1ms时间  18kpwm
        get_num++;
        if(get_num >= 14)
        {
            get_num = 0;
            g_bldc_time.g_time_sys++;
            g_bldc_time.g_time_task1++;
            g_bldc_time.g_time_task2++;
            g_bldc_time.g_time_task3++;
            g_bldc_time.g_time_task4++;
        }
        
        ADC1->CR2 |= (ADC_CR2_SWSTART);
//        ADC1->CR2 |= (ADC_CR2_SWSTART);
        
        //霍尔检查
        g_bldc_motorA.step_sta = hallsensor_get_state(MOTORA);
        g_bldc_motorB.step_sta = hallsensor_get_state(MOTORB);
        
        //霍尔处理
        hall_judge(&g_bldc_motorA,&MA_step_ch);
        hall_judge(&g_bldc_motorB,&MB_step_ch);
        
        
        //换向函数
        
        /*******     刹车，主要要一起处理，不然一个电机用电阻刹车，一个用电流刹车就短路了      ******/
        if(g_bldc_motorA.brake_flag > 0 || g_bldc_motorB.brake_flag > 0)
        {
            g_bldc_motorA.run_flag = STOP;
            g_bldc_motorB.run_flag = STOP;
            
            //霍尔故障，高速都用电机内阻刹车
            if( g_bldc_motorA.speed >= 300 || g_bldc_motorB.speed >= 300 || \
                (g_bldc_motorA.hall_miss == SET && g_bldc_motorB.hall_miss == SET))
            {
                
                g_bldc_motorA.brake_flag = 1;
                MX_power(0);
                
                //上桥用GPIO控制，下桥用AF模式，CCR的值就是控制下桥了
                if(bk_mode != 1)
                {
                    bk_mode = 1;
                    MA_H_afmode(0);
                    MA_L_afmode(1);
                    MB_H_afmode(0);
                    MB_L_afmode(1);
                    //改变频率
                    TIM1->PSC = HZ_P_BK-1;
                    TIM8->PSC = HZ_P_BK-1;
                }
                MA_break();
                MB_break();
                
            }else if( g_bldc_motorA.speed < 250 && g_bldc_motorB.speed < 250 )
            {
                uint8_t sta_temp = 0;
                static uint8_t ma_bk_sta = 1;
                static uint8_t mb_bk_sta = 1;
                
                g_bldc_motorA.brake_flag = 2;
                MX_power(1);
                //上桥用AF，下桥GP
                if(bk_mode != 2)
                {
                    bk_mode = 2;
                    MA_H_afmode(1);
                    MA_L_afmode(0);
                    MB_H_afmode(1);
                    MB_L_afmode(0);
                    //改变频率
                    TIM1->PSC = HZ_P_BK-1;
                    TIM8->PSC = HZ_P_BK-1;
                    g_bldc_motorA.brake_duty = 0;
                    g_bldc_motorB.brake_duty = 0;
                }
                
                //低速用低占空比刹车
                
                sta_temp = brake_cun_set(&g_bldc_motorA);
                if(sta_temp >= 1 && sta_temp <= 4)
                {
                    ma_bk_sta = sta_temp;
                }
                pfunclist_motorA_bk[ma_bk_sta - 1]();
                
                sta_temp = brake_cun_set(&g_bldc_motorB);
                if(sta_temp >= 1 && sta_temp <= 4)
                {
                    mb_bk_sta = sta_temp;
                }
                pfunclist_motorB_bk[mb_bk_sta - 1]();
            }
            
        }else
        {
            //通电
            MX_power(1);
            
            //上桥用AF，下桥GP
            if(bk_mode != 0)
            {
                bk_mode = 0;
                MA_H_afmode(1);
                MA_L_afmode(0);
                MB_H_afmode(1);
                MB_L_afmode(0);
                //改变频率
                TIM1->PSC = HZ_P_RUN-1;
                TIM8->PSC = HZ_P_RUN-1;
            }
            
            //motora换向处理
            if(g_bldc_motorA.run_flag == STOP || g_bldc_motorA.hall_miss == SET)
            {
                MA_stop();
            }else if(g_bldc_motorA.run_flag == RUN)
            {
                if(g_bldc_motorA.step_sta > 0 && g_bldc_motorA.step_sta < 7)
                {
                    if(g_bldc_motorA.setdir == CCW)                                     /* 反转 */
                    {
                        pfunclist_motorA_ccw[g_bldc_motorA.step_sta - 1]();
                    }else
                    {
                        pfunclist_motorA_cw[g_bldc_motorA.step_sta - 1]();
                    }
                }
            }
            
            //motorb换向处理
            if(g_bldc_motorB.run_flag == STOP || g_bldc_motorB.hall_miss == SET)
            {
                MB_stop();
            }else if(g_bldc_motorB.run_flag == RUN)
            {
                if(g_bldc_motorB.step_sta > 0 && g_bldc_motorB.step_sta < 7)
                {
                    if(g_bldc_motorB.setdir == CCW)                                     /* 反转 */
                    {
                        pfunclist_motorB_ccw[g_bldc_motorB.step_sta - 1]();
                    }else
                    {
                        pfunclist_motorB_cw[g_bldc_motorB.step_sta - 1]();
                    }
                }
            }
            
        }
        
        
    }

}
