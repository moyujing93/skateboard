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
 
#include "./BSP/bldc_six_step.h"
#include "./BSP/bldc.h"


# define    FD6288T     1

/**
  * @brief  设置GPIO的AF模式与普通GPIO模式
  * @param  1 = AF-MODE   0 = GPIO-MODE
  * @retval 无
  */
void MA_H_afmode(uint8_t sta)
{
    if(sta)
    {
        MA_UH_PWM_PORT->CRL |=  ((uint32_t)0x08 << 24);
        
        
        MA_VH_PWM_PORT->CRL |=  ((uint32_t)0x08 << 28);
        
        
        MA_WH_PWM_PORT->CRH |=  ((uint32_t)0x08 << 0);
    }else
    {
        MA_UH_PWM_PORT->CRL &= ~((uint32_t)0x08 << 24);
        
        
        MA_VH_PWM_PORT->CRL &= ~((uint32_t)0x08 << 28);
        
        
        MA_WH_PWM_PORT->CRH &= ~((uint32_t)0x08 << 0);
    }
}
/**
  * @brief  设置GPIO的AF模式与普通GPIO模式
  * @param  1 = AF-MODE   0 = GPIO-MODE
  * @retval 无
  */
void MA_L_afmode(uint8_t sta)
{
    if(sta)
    {
        MA_UL_PORT->CRL |=  ((uint32_t)0x08 << 28);
        
        
        MA_VL_PORT->CRL |=  ((uint32_t)0x08 << 0);
        
        
        MA_WL_PORT->CRL |=  ((uint32_t)0x08 << 4);
    }else
    {
        MA_UL_PORT->CRL &=  ~((uint32_t)0x08 << 28);
        
        
        MA_VL_PORT->CRL &=  ~((uint32_t)0x08 << 0);
        
        
        MA_WL_PORT->CRL &=  ~((uint32_t)0x08 << 4);
    }
}

/**
  * @brief  设置GPIO的AF模式与普通GPIO模式
  * @param  1 = AF-MODE   0 = GPIO-MODE
  * @retval 无
  */
void MB_H_afmode(uint8_t sta)
{
    if(sta)
    {
        MB_UH_PWM_PORT->CRH |=  ((uint32_t)0x08 << 0);
        
        
        MB_VH_PWM_PORT->CRH |=  ((uint32_t)0x08 << 4);
        
        
        MB_WH_PWM_PORT->CRH |=  ((uint32_t)0x08 << 8);
    }else
    {
        MB_UH_PWM_PORT->CRH &= ~((uint32_t)0x08 << 0);
        
        
        MB_VH_PWM_PORT->CRH &= ~((uint32_t)0x08 << 4);
        
        
        MB_WH_PWM_PORT->CRH &= ~((uint32_t)0x08 << 8);
    }
}

/**
  * @brief  设置GPIO的AF模式与普通GPIO模式
  * @param  1 = AF-MODE   0 = GPIO-MODE
  * @retval 无
  */
void MB_L_afmode(uint8_t sta)
{
    if(sta)
    {
        MB_UL_PORT->CRH |=  ((uint32_t)0x08 << 20);
        
        
        MB_VL_PORT->CRH |=  ((uint32_t)0x08 << 24);
        
        
        MB_WL_PORT->CRH |=  ((uint32_t)0x08 << 28);
    }else
    {
        MB_UL_PORT->CRH &=  ~((uint32_t)0x08 << 20);
        
        
        MB_VL_PORT->CRH &=  ~((uint32_t)0x08 << 24);
        
        
        MB_WL_PORT->CRH &=  ~((uint32_t)0x08 << 28);
    }
}




//桥驱动芯片FD6288T，自带死区控制，防止上下桥同时导通
//HIN = 1，LIN=0 ：HO=1,LO=0
//HIN = 0，LIN=1 ：HO=0,LO=1
//其他状态       ：HO=0,LO=0
#if FD6288T



/**********************************MOTOR A**************************************/

/**
  * @brief  U相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_uhvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;
    g_MA_timx_handle.Instance->CCR2 = 0;
    g_MA_timx_handle.Instance->CCR3 = 0;
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  U相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_uhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;
    g_MA_timx_handle.Instance->CCR2 = 0;
    g_MA_timx_handle.Instance->CCR3 = 0;
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_vhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_vhul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_whul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_whvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}


/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void MA_stop(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  刹车
  * @param  无
  * @retval 无
  */
void MA_break(void)
{
    /******* 上桥臂PWM部分 ********/
    MA_UH_PWM_PORT->BSRR = (uint32_t)MA_UH_PWM_PIN << 16u;
    MA_VH_PWM_PORT->BSRR = (uint32_t)MA_VH_PWM_PIN << 16u;
    MA_WH_PWM_PORT->BSRR = (uint32_t)MA_WH_PWM_PIN << 16u;
    
    if(g_bldc_motorA.brake_mode == 1)  //死区补偿
    {
        if(g_bldc_motorA.brake_duty <= 950)  //电流太大退出补偿
        {
            MA_L_afmode(1);
            g_bldc_motorA.brake_mode = 0;
            return;
        }
        MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
        MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
        MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
        
    }else
    {
        if(g_bldc_motorA.brake_duty >= MAX_PWM_BRAKE_SET)  //进入补偿
        {
            MA_L_afmode(0);
            g_bldc_motorA.brake_mode = 1;
            return;
        }
        g_MA_timx_handle.Instance->CCR1 = (1000 - g_bldc_motorA.brake_duty);    //上桥输出高电平互补通道就输出低电平
        g_MA_timx_handle.Instance->CCR2 = (1000 - g_bldc_motorA.brake_duty);
        g_MA_timx_handle.Instance->CCR3 = (1000 - g_bldc_motorA.brake_duty);
        
    }
    
}


/**********************************MOTOR B**************************************/


/**
  * @brief  U相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_uhvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  U相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_uhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_vhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_vhul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;
    g_MB_timx_handle.Instance->CCR3 = 0;
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_whul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_whvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;
    g_MB_timx_handle.Instance->CCR2 = 0;
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}


/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void MB_stop(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  刹车
  * @param  无
  * @retval 无
  */
void MB_break(void)
{
    /******* 上桥臂PWM部分 ********/
    MB_UH_PWM_PORT->BSRR = (uint32_t)MB_UH_PWM_PIN << 16u;
    MB_VH_PWM_PORT->BSRR = (uint32_t)MB_VH_PWM_PIN << 16u;
    MB_WH_PWM_PORT->BSRR = (uint32_t)MB_WH_PWM_PIN << 16u;
    
    if(g_bldc_motorB.brake_mode == 1)  //死区补偿
    {
        if(g_bldc_motorB.brake_duty <= 950)  //电流太大退出补偿
        {
            MB_L_afmode(1);
            g_bldc_motorB.brake_mode = 0;
            return;
        }
        MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
        MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
        MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
        
    }else
    {
        if(g_bldc_motorB.brake_duty >= MAX_PWM_BRAKE_SET)  //进入补偿
        {
            MB_L_afmode(0);
            g_bldc_motorB.brake_mode = 1;
            return;
        }
        g_MB_timx_handle.Instance->CCR1 = (1000 - g_bldc_motorB.brake_duty);    //上桥输出高电平互补通道就输出低电平
        g_MB_timx_handle.Instance->CCR2 = (1000 - g_bldc_motorB.brake_duty);
        g_MB_timx_handle.Instance->CCR3 = (1000 - g_bldc_motorB.brake_duty);
        
    }
    
}


#endif

