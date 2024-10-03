/**
 ****************************************************************************************************
 * @file        bldc.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/bldc.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/bldc_six_step.h"


volatile uint32_t time_num = 0;

volatile uint8_t MA_hall_speed_sta = 0x01;   // 120�Ȼ���״̬��СֵΪ1��60��Ϊ0
volatile uint8_t MB_hall_speed_sta = 0x01;   // 120�Ȼ���״̬��СֵΪ1��60��Ϊ0

/*  ����������ָ������ */
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

/* ����ṹ���ʼֵ */
_bldc_obj g_bldc_motorA = {0};
_bldc_obj g_bldc_motorB = {0};

/* HAL������ṹ�� */
TIM_HandleTypeDef   g_MA_timx_handle;
TIM_HandleTypeDef   g_MB_timx_handle;


/**
 * @brief       �����޷�����
 * @param       ����
 * @param       ��Сֵ
 * @param       ���ֵ
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


/***************************************** �������� *************************************************/
/**
 * @brief       ��ˢ�����ʼ����������ʱ���������ӿ����ų�ʼ��
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void bldc_init(uint16_t arr, uint16_t psc)
{
    TIM_BreakDeadTimeConfigTypeDef g_sbreak_dead_time_config;
    TIM_OC_InitTypeDef  g_atimx_oc_chy_handle;
    GPIO_InitTypeDef gpio_init_struct;
    
    /**********************��ʱ��************************/
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    
    
    /*********************MOTORA*************************/
    
    /* MOTORB���ű����ų�ʼ�� */
    gpio_init_struct.Pin = MA_UH_PWM_PIN | MA_VH_PWM_PIN | MA_WH_PWM_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    
    /* ���ű�TIM��ʼ�� */
    g_MA_timx_handle.Instance = TIM8;
    g_MA_timx_handle.Init.Prescaler = psc;                        /* ��ʱ����Ƶ */
    g_MA_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ����ģʽ */
    g_MA_timx_handle.Init.Period = arr;                           /* �Զ���װ��ֵ */
    g_MA_timx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR��Ƶ���� */
    g_MA_timx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_MA_timx_handle.Init.RepetitionCounter = 0;                  /* �ظ�����*/
    HAL_TIM_PWM_Init(&g_MA_timx_handle);                          /* ��ʼ��PWM */
    
    /* ��������Ƚ�ģʽ */
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* ģʽѡ�� */
    g_atimx_oc_chy_handle.Pulse = 0;                            /* �Ƚ�ֵ */
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* ���� */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCPOLARITY_HIGH;   /* ����ͨ������ */
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_ENABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MA_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    

    /* �����������������������ж� */
//    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* ����ģʽ�Ĺر����״̬ */
//    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* ����ģʽ�Ĺر����״̬ */
//    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* ���üĴ��������� */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* ʹ��ɲ������ */
    __HAL_TIM_MOE_ENABLE(&g_MA_timx_handle);  /* MOE=1,ʹ������� */
    g_sbreak_dead_time_config.DeadTime = 0x3f;       /* ����ʱ������ */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MA_timx_handle, &g_sbreak_dead_time_config);
    
    /* ������ʱ����� */
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 2, 0);               /* ���ȼ��� */
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    HAL_TIM_Base_Start_IT(&g_MA_timx_handle);                 /* ����ʱ����Ԫ�ĸ����ж� */
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MA_timx_handle,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MA_timx_handle,TIM_CHANNEL_3);


    /* ���ű����ų�ʼ�� */
    gpio_init_struct.Pin = MA_VL_PIN | MA_WL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    gpio_init_struct.Pin = MA_UL_PIN;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    /* �������ų�ʼ�� */
    gpio_init_struct.Pin = MA_HALL_U_PIN | MA_HALL_V_PIN | MA_HALL_W_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
    
    
    /*********************MOTORB*************************/
    
    /* MOTORB���ű����ų�ʼ�� */
    gpio_init_struct.Pin = MB_UH_PWM_PIN | MB_VH_PWM_PIN | MB_WH_PWM_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    /* ���ű�TIM��ʼ�� */
    g_MB_timx_handle.Instance = TIM1;
    g_MB_timx_handle.Init.Prescaler = psc;                        /* ��ʱ����Ƶ */
    g_MB_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ����ģʽ */
    g_MB_timx_handle.Init.Period = arr;                           /* �Զ���װ��ֵ */
    g_MB_timx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ETR��Ƶ���� */
    g_MB_timx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    g_MB_timx_handle.Init.RepetitionCounter = 0;                  /* �ظ�����*/
    HAL_TIM_PWM_Init(&g_MB_timx_handle);                          /* ��ʼ��PWM */
    
    /* ��������Ƚ�ģʽ */
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* ģʽѡ�� */
    g_atimx_oc_chy_handle.Pulse = 0;                            /* �Ƚ�ֵ */
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* ���� */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCPOLARITY_HIGH;   /* ����ͨ������ */
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_ENABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&g_MB_timx_handle,&g_atimx_oc_chy_handle,TIM_CHANNEL_3);
    
    /* �����������������������ж� */
//    g_sbreak_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* ����ģʽ�Ĺر����״̬ */
//    g_sbreak_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* ����ģʽ�Ĺر����״̬ */
//    g_sbreak_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* ���üĴ��������� */
    g_sbreak_dead_time_config.BreakState = TIM_BREAK_DISABLE;                /* ʹ��ɲ������ */
    __HAL_TIM_MOE_ENABLE(&g_MB_timx_handle);  /* MOE=1,ʹ������� */
    g_sbreak_dead_time_config.DeadTime = 0x3f;       /* ����ʱ������ */
    HAL_TIMEx_ConfigBreakDeadTime(&g_MB_timx_handle, &g_sbreak_dead_time_config);
    
    /* ������ʱ����� */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);               /* ���ȼ��� */
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_TIM_Base_Start_IT(&g_MB_timx_handle);                 /* ����ʱ����Ԫ�ĸ����ж� */
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_MB_timx_handle,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&g_MB_timx_handle,TIM_CHANNEL_3);


    /* ���ű����ų�ʼ�� */
    gpio_init_struct.Pin = MB_UL_PIN | MB_VL_PIN | MB_WL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
    /* �������ų�ʼ�� */
    gpio_init_struct.Pin = MB_HALL_U_PIN | MB_HALL_V_PIN | MB_HALL_W_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    
    
}

/**
 * @brief       ��ȡ��������������״̬
 * @param       
 * @retval      ��������������״̬
 */
static uint8_t hallsensor_get_state(motornum_Type  motor_num)
{
    /*
            HU           HV         HW
            λ2          λ1        λ0
    */
    uint8_t state = 0x00U;
    
    if(motor_num == MOTORA)
    {
        if(HAL_GPIO_ReadPin(MA_HALL_U_PORT,MA_HALL_U_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x04U;
        }
        if(HAL_GPIO_ReadPin(MA_HALL_V_PORT,MA_HALL_V_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(MA_HALL_W_PORT,MA_HALL_W_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x01U;
        }
    }else if(motor_num == MOTORB)
    {
        if(HAL_GPIO_ReadPin(MB_HALL_U_PORT,MB_HALL_U_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x04U;
        }
        if(HAL_GPIO_ReadPin(MB_HALL_V_PORT,MB_HALL_V_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(MB_HALL_W_PORT,MB_HALL_W_PIN) == GPIO_PIN_SET)  /* ����������״̬��ȡ */
        {
            state |= 0x01U;
        }
    }

    
    /* 60�Ȼ���ʹ�� */
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
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
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
        /*          ���ຯ��        */
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
            /*          �������        */
            if(g_bldc_motorA.dir == CCW)                                     /* ��ת */
            {
                g_bldc_motorA.step_sta = 7 - hallsensor_get_state(MOTORA);
            }
            else
            {
                g_bldc_motorA.step_sta = hallsensor_get_state(MOTORA);
            }
            
            if((g_bldc_motorA.step_sta <= 6)&&(g_bldc_motorA.step_sta >= 1))/* �жϻ������ֵ�Ƿ����� */
            {
                pfunclist_motorA[g_bldc_motorA.step_sta - 1]();                   /* ͨ�������Ա���Ҷ�Ӧ�ĺ���ָ�� */
            }
            
            /*          �����ٶȼ������        */
            if(g_bldc_motorA.step_sta == MA_hall_speed_sta)/* ���˵������������ */
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
        /*          ���ຯ��        */
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
            /*          �������        */
            if(g_bldc_motorB.dir == CCW)                                     /* ��ת */
            {
                g_bldc_motorB.step_sta = 7 - hallsensor_get_state(MOTORB);
            }
            else
            {
                g_bldc_motorB.step_sta = hallsensor_get_state(MOTORB);
            }
            
            if((g_bldc_motorB.step_sta <= 6)&&(g_bldc_motorB.step_sta >= 1))/* �жϻ������ֵ�Ƿ����� */
            {
                pfunclist_motorB[g_bldc_motorB.step_sta - 1]();                   /* ͨ�������Ա���Ҷ�Ӧ�ĺ���ָ�� */
            }
            
            /*          �����ٶȼ������        */
            if(g_bldc_motorB.step_sta == MB_hall_speed_sta)/* ���˵������������ */
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
