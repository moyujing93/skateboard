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


uint16_t break_motorA = 0;
uint16_t break_motorB = 0;

# define    FD6288T     1

# define    IRS2003     0


/**
  * @brief  AF模式和output模式得切换
  * @param  无
  * @retval 无
  */
void gpio_afmode(uint8_t ch)
{
  if(ch == 1)
  {
    GPIOB->CRH &= ~(0xC0000000 >> 8);
    GPIOB->CRH |=  (0x80000000 >> 8);
  }else if(ch == 2)
  {
    GPIOB->CRH &= ~(0xC0000000 >> 4);
    GPIOB->CRH |=  (0x80000000 >> 4);
  }else if(ch == 3)
  {
    GPIOB->CRH &= ~0xC0000000;
    GPIOB->CRH |=  0x80000000;
  }
  
}
/**
  * @brief  AF模式和output模式得切换
  * @param  无
  * @retval 无
  */
void gpio_outputmode(uint8_t ch)
{
  if(ch == 1)
  {
    GPIOB->CRH &= ~(0xC0000000 >> 8);
  }else if(ch == 2)
  {
    GPIOB->CRH &= ~(0xC0000000 >> 4);
  }else if(ch == 3)
  {
    GPIOB->CRH &= ~0xC0000000;
  }
}

/**
  * @brief  AF模式和output模式得切换
  * @param  无
  * @retval 无
  */
void gpio_outputmodeuuuu(uint8_t ch)
{
  if(ch == 1)
  {
    GPIOA->CRH &= ~(0xC0000000 >> 18);
  }else if(ch == 2)
  {
    GPIOA->CRH &= ~(0xC0000000 >> 14);
  }else if(ch == 3)
  {
    GPIOA->CRH &= ~(0xC0000000 >> 10);
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
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
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
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
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
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
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
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
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
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
        /******* 下桥臂GPIO部分 ******/
    if(break_motorA < 17)
    {
        break_motorA++;
    }else
    {
        break_motorA = 0;
    }
    if(break_motorA < g_bldc_motorA.brake_duty)
    {
        MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
        MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
        MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    }else
    {
        MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
        MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
        MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
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
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
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
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
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
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;                        /* W相上桥臂 */
    
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
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    if(break_motorB < 17)
    {
        break_motorB++;
    }else
    {
        break_motorB = 0;
    }
    if(break_motorB < g_bldc_motorB.brake_duty)
    {
        MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
        MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
        MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    }else
    {
        MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
        MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
        MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    }
    
}


#endif



//桥驱动芯片IRS2003
//HIN = 1，LIN=1 ：HO=1,LO=0
//HIN = 0，LIN=0 ：HO=0,LO=1
//其他状态       ： HO=0,LO=0
#if   IRS2003


/**********************************MOTOR A**************************************/
/**
  * @brief  U相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_uhvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  U相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_uhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_vhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;    /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_vhul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;    /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_whul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MA_whvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}


/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void MA_stop(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MA_timx_handle.Instance->CCR3 = 0;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
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
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
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
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_vhwl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;    /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_vhul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;    /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_whul(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void MB_whvl(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}


/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void MB_stop(void)
{
    /******* 上桥臂PWM部分 ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U相上桥臂 */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V相上桥臂 */
    g_MB_timx_handle.Instance->CCR3 = 0;   /* W相上桥臂 */
    
    /******* 下桥臂GPIO部分 ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}


#endif

