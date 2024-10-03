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
 
#include "./BSP/bldc_six_step.h"
#include "./BSP/bldc.h"


uint16_t break_motorA = 0;
uint16_t break_motorB = 0;

# define    FD6288T     1

# define    IRS2003     0


/**
  * @brief  AFģʽ��outputģʽ���л�
  * @param  ��
  * @retval ��
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
  * @brief  AFģʽ��outputģʽ���л�
  * @param  ��
  * @retval ��
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
  * @brief  AFģʽ��outputģʽ���л�
  * @param  ��
  * @retval ��
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



//������оƬFD6288T���Դ��������ƣ���ֹ������ͬʱ��ͨ
//HIN = 1��LIN=0 ��HO=1,LO=0
//HIN = 0��LIN=1 ��HO=0,LO=1
//����״̬       ��HO=0,LO=0
#if FD6288T



/**********************************MOTOR A**************************************/

/**
  * @brief  U�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_uhvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  U�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_uhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  V�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_vhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  V�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_vhul(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  W�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_whul(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  W�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_whvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}


/**
  * @brief  �رյ����ת
  * @param  ��
  * @retval ��
  */
void MA_stop(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  ɲ��
  * @param  ��
  * @retval ��
  */
void MA_break(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
        /******* ���ű�GPIO���� ******/
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
  * @brief  U�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_uhvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  U�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_uhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  V�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_vhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  V�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_vhul(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  W�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_whul(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  W�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_whvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}


/**
  * @brief  �رյ����ת
  * @param  ��
  * @retval ��
  */
void MB_stop(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  ɲ��
  * @param  ��
  * @retval ��
  */
void MB_break(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
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



//������оƬIRS2003
//HIN = 1��LIN=1 ��HO=1,LO=0
//HIN = 0��LIN=0 ��HO=0,LO=1
//����״̬       �� HO=0,LO=0
#if   IRS2003


/**********************************MOTOR A**************************************/
/**
  * @brief  U�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_uhvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  U�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_uhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = g_bldc_motorA.pwm_duty;   /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  V�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_vhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;    /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN << 16u;
    
}

/**
  * @brief  V�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_vhul(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = g_bldc_motorA.pwm_duty;    /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  W�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_whul(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN << 16u;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}

/**
  * @brief  W�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MA_whvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = g_bldc_motorA.pwm_duty;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN << 16u;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}


/**
  * @brief  �رյ����ת
  * @param  ��
  * @retval ��
  */
void MA_stop(void)
{
    /******* ���ű�PWM���� ********/
    g_MA_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MA_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MA_timx_handle.Instance->CCR3 = 0;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MA_UL_PORT->BSRR = (uint32_t)MA_UL_PIN;
    MA_VL_PORT->BSRR = (uint32_t)MA_VL_PIN;
    MA_WL_PORT->BSRR = (uint32_t)MA_WL_PIN;
    
}



/**********************************MOTOR B**************************************/
/**
  * @brief  U�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_uhvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  U�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_uhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = g_bldc_motorB.pwm_duty;   /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  V�����ű۵�ͨ��W�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_vhwl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;    /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN << 16u;
    
}

/**
  * @brief  V�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_vhul(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = g_bldc_motorB.pwm_duty;    /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;                        /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  W�����ű۵�ͨ��U�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_whul(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN << 16u;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}

/**
  * @brief  W�����ű۵�ͨ��V�����ű۵�ͨ
  * @param  ��
  * @retval ��
  */
void MB_whvl(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = g_bldc_motorB.pwm_duty;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN << 16u;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}


/**
  * @brief  �رյ����ת
  * @param  ��
  * @retval ��
  */
void MB_stop(void)
{
    /******* ���ű�PWM���� ********/
    g_MB_timx_handle.Instance->CCR1 = 0;                        /* U�����ű� */
    g_MB_timx_handle.Instance->CCR2 = 0;                        /* V�����ű� */
    g_MB_timx_handle.Instance->CCR3 = 0;   /* W�����ű� */
    
    /******* ���ű�GPIO���� ******/
    MB_UL_PORT->BSRR = (uint32_t)MB_UL_PIN;
    MB_VL_PORT->BSRR = (uint32_t)MB_VL_PIN;
    MB_WL_PORT->BSRR = (uint32_t)MB_WL_PIN;
    
}


#endif

