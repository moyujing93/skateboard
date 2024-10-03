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


volatile uint32_t time_num = 0;

volatile uint8_t MA_hall_speed_sta = 0x01;   // 120度霍尔状态最小值为1，60度为0
volatile uint8_t MB_hall_speed_sta = 0x01;   // 120度霍尔状态最小值为1，60度为0

/*  六步换向函数指针数组 */
pctr pfunclist_motorA[6] =
{
    &MA_whvl,&MA_vhul,&MA_whul,
    &MA_uhwl,&MA_uhvl,&MA_vhwl
};

pctr pfunclist_motorB[6] =
{
    &MB_whvl,&MB_vhul,&MB_whul,
    &MB_uhwl,&MB_uhvl,&MB_vhwl
};

/* 电机结构体初始值 */
_bldc_obj g_bldc_motorA = {0};
_bldc_obj g_bldc_motorB = {0};

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
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_ENABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    

    /* 设置死区参数，开启死区中断 */
//    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
//    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
//    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    __HAL_TIM_MOE_ENABLE(&g_MA_timx_handle);  /* MOE=1,使能主输出 */
    g_sbreak_dead_time_config.DeadTime = 0x3f;       /* 死区时间设置 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MA_timx_handle, &g_sbreak_dead_time_config);
    
    /* 开启定时器输出 */
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 2, 0);               /* 优先级高 */
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    HAL_TIM_Base_Start_IT(&g_MA_timx_handle);                 /* 启动时基单元的更新中断 */
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_3);


    /* 下桥臂引脚初始化 */
    gpio_init_struct.Pin = MA_VL_PIN | MA_WL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    gpio_init_struct.Pin = MA_UL_PIN;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
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
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_ENABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    
    /* 设置死区参数，开启死区中断 */
//    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
//    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
//    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    __HAL_TIM_MOE_ENABLE(&g_MB_timx_handle);  /* MOE=1,使能主输出 */
    g_sbreak_dead_time_config.DeadTime = 0x3f;       /* 死区时间设置 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MB_timx_handle, &g_sbreak_dead_time_config);
    
    /* 开启定时器输出 */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);               /* 优先级高 */
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_TIM_Base_Start_IT(&g_MB_timx_handle);                 /* 启动时基单元的更新中断 */
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_3);


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
            state |= 0x04U;
        }
        if(HAL_GPIO_ReadPin(MA_HALL_V_PORT,MA_HALL_V_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(MA_HALL_W_PORT,MA_HALL_W_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x01U;
        }
    }else if(motor_num == MOTORB)
    {
        if(HAL_GPIO_ReadPin(MB_HALL_U_PORT,MB_HALL_U_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x04U;
        }
        if(HAL_GPIO_ReadPin(MB_HALL_V_PORT,MB_HALL_V_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(MB_HALL_W_PORT,MB_HALL_W_PIN) == GPIO_PIN_SET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x01U;
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
 * @brief       定时器中断服务函数
 * @param       无
 * @retval      无
 */
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_MB_timx_handle);
}

void TIM8_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_MA_timx_handle);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8)
    {
        /*          换相函数        */
        if(g_bldc_motorA.run_flag == STOP)
        {
            if(g_bldc_motorA.brake_flag == 1)
            {
                MA_break();
            }else
            {
                MA_stop();
            }
            
        }else if(g_bldc_motorA.run_flag == RUN)
        {
            /*          霍尔检测        */
            if(g_bldc_motorA.dir == CCW)                                     /* 反转 */
            {
                g_bldc_motorA.step_sta = 7 - hallsensor_get_state(MOTORA);
            }
            else
            {
                g_bldc_motorA.step_sta = hallsensor_get_state(MOTORA);
            }
            
            if((g_bldc_motorA.step_sta <= 6)&&(g_bldc_motorA.step_sta >= 1))/* 判断霍尔组合值是否正常 */
            {
                pfunclist_motorA[g_bldc_motorA.step_sta - 1]();                   /* 通过数组成员查找对应的函数指针 */
            }
            
            /*          增加速度计算参数        */
            if(g_bldc_motorA.step_sta == MA_hall_speed_sta)/* 相等说明走完三步了 */
            {
                g_bldc_motorA.hall_speed_num++;
                if(MA_hall_speed_sta == 5)
                {
                   MA_hall_speed_sta = 1;
                }
                else
                {
                    MA_hall_speed_sta = 5;
                }
            }
        }
        
    }else if(htim->Instance == TIM1)
    {
        /*          换相函数        */
        if(g_bldc_motorB.run_flag == STOP)
        {
            if(g_bldc_motorB.brake_flag == 1)
            {
                MB_break();
            }else
            {
                MB_stop();
            }
        }else if(g_bldc_motorB.run_flag == RUN)
        {
            /*          霍尔检测        */
            if(g_bldc_motorB.dir == CCW)                                     /* 反转 */
            {
                g_bldc_motorB.step_sta = 7 - hallsensor_get_state(MOTORB);
            }
            else
            {
                g_bldc_motorB.step_sta = hallsensor_get_state(MOTORB);
            }
            
            if((g_bldc_motorB.step_sta <= 6)&&(g_bldc_motorB.step_sta >= 1))/* 判断霍尔组合值是否正常 */
            {
                pfunclist_motorB[g_bldc_motorB.step_sta - 1]();                   /* 通过数组成员查找对应的函数指针 */
            }
            
            /*          增加速度计算参数        */
            if(g_bldc_motorB.step_sta == MB_hall_speed_sta)/* 相等说明走完三步了 */
            {
                g_bldc_motorB.hall_speed_num++;
                if(MB_hall_speed_sta == 5)
                {
                   MB_hall_speed_sta = 1;
                }
                else
                {
                    MB_hall_speed_sta = 5;
                }
            }
        }
        
    }
}
