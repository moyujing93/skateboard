 /*======================##### BLDC�и���ˢ���� #####=========================
 ʹ�õ����� 
 RS485(USART1)     PB5_R/D(GPIO),PB6_TX(AFIO),PB7_RX(AFIO)
 ����(TIM1)        (��ͨ��)PA8,9,10.(����ͨ��)PB13,14,15
 HALL(EXTI5,6,7)      PA7,PB0,PB1
 �������(ADC1)        PA0,1,2
 ���ߵ�ѹ(ADC1)        PA4
 �������¶�(ADC1)      PA5
 pid��ʱ����(TIM2)     NULL
 ===============================================================================*/
#include "stm32f1xx.h"
#include <string.h>
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/bldc.h"
#include "./BSP/pid.h"
#include "./BSP/adc.h"
#include "./BSP/rs485.h"
#include "./BSP/pulse.h"



#define USE_RC     0

uint16_t time_max_c = 0;
uint16_t time_locked_rotor = 0;

void over_load(_bldc_obj *motor_temp);

int main(void)
{
    HAL_Init();                                 /* ���ȼ�����Ϊ4λ��ռʽ */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* ����ʱ��,72M */
    delay_init(72);                             /* ��ʼ����ʱ���� */
    
    //�ص�JTAG����ʹ��PB3,4
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    
    //ʹ��RCң��Э��͵ùر�RS485
    #if USE_RC
    pulse_init();
    #else
    rs485_init(115200);
    #endif
    delay_ms(100);
    
    
    g_bldc_motorA.pwm_duty = RESET;
    g_bldc_motorA.run_flag = STOP;
    g_bldc_motorA.dir = CCW;
    g_bldc_motorA.max_c = RESET;
    g_bldc_motorA.max_t = RESET;
    
    
    g_bldc_motorB.pwm_duty = RESET;
    g_bldc_motorB.run_flag = STOP;
    g_bldc_motorB.dir = CW;
    g_bldc_motorB.max_c = RESET;
    g_bldc_motorB.max_t = RESET;
    
                    
    bldc_init(1000-1,4-1);
    delay_ms(1000);
    
    adc1_dma_init();
    pid_init(200);
    
    
    while(1)
    {
        
        if( time_num % 10 == 0)
        {
            #if USE_RC
            /* RCͨѶ */
            pulse_fetinst(0) ;
            #else
            /* wifi_now */
            ESP32_fetinst(2);
            #endif
        }
        
        if( time_num % 5 == 0)
        {
            /* ����������¶� */
            adc_dma_conver(20);
        }
        
        if( time_num % 20 == 0)
        {
            over_load(&g_bldc_motorA);
            over_load(&g_bldc_motorB);
        }
        #define   rpm_time    20
        if( time_num % rpm_time == 0)
        {
            /* ����ת�� */
            
            /*RPM/MIN = ��get_speed_num/������/2�� * (60000/ time) 
            ����: ��������ʱ��=100ms ,����ֵ=50 ������=7;
            RPM/min = (50/14)*��60000ms/50ms�� = 4285rpm/min
            */
            g_bldc_motorA.speed = 0.3f * ((g_bldc_motorA.hall_speed_num * (60000/rpm_time)) / 14) + 0.7f * g_bldc_motorA.speed;
            g_bldc_motorA.hall_speed_num = 0;
            
            g_bldc_motorB.speed = 0.3f * ((g_bldc_motorB.hall_speed_num * (60000/rpm_time)) / 14) + 0.7f * g_bldc_motorB.speed;
            g_bldc_motorB.hall_speed_num = 0;
            
        }
        
        
        
        time_num ++;
        delay_ms(1);
    }
}




void over_load(_bldc_obj *motor_temp)
{
    //�¶ȱ������ͷ�
    if(motor_temp->v_t  >  HOT_OTP)
    {
        motor_temp->max_t = SET;
    }else if(motor_temp->v_t < (HOT_OTP-2000))
    {
        motor_temp->max_t = RESET;
    }
    
    
    //�����������ͷ�
    if(motor_temp->current > MAX_CURRENT)
    {
        motor_temp->max_c = SET;
        time_locked_rotor++;
        if(time_locked_rotor > 10)
        {
            motor_temp->locked_rotor = SET;
        }
    }else
    {
        time_max_c ++;
        if(time_max_c > 25)
        {
            time_max_c = 0;
            time_locked_rotor = 0;
            motor_temp->max_c = RESET;
            motor_temp->locked_rotor = RESET;
        }
    }
}
/*
 �ѵ���ȥ��
 ��Ӣ�忪��RS485����MDR

*/

