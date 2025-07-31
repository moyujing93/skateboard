 /*======================##### BLDC有感无刷驱动 #####=========================
 使用的外设 
 RS485(USART1)     PB5_R/D(GPIO),PB6_TX(AFIO),PB7_RX(AFIO)
 驱动(TIM1)        (主通道)PA8,9,10.(互补通道)PB13,14,15
 HALL(EXTI5,6,7)      PA7,PB0,PB1
 三相电流(ADC1)        PA0,1,2
 主线电压(ADC1)        PA4
 驱动板温度(ADC1)      PA5
 pid定时计算(TIM2)     NULL
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



#define USE_RC     0




IWDG_HandleTypeDef g_iwdg_handle;  /* 独立看门狗句柄 */



void over_load(_bldc_obj *motor_temp);

int main(void)
{
    HAL_Init();                                 /* 优先级分组为4位抢占式 */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟,72M */
    delay_init(72);                             /* 初始化延时函数 */
	//启动测试
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    //获取时间
	DWT->CYCCNT = 0;
	if(TEST_SYS_TICK < DWT->CYCCNT) TEST_SYS_TICK = DWT->CYCCNT;
    
    //关掉JTAG用于使用PB3,4
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    
    //使用RC遥控协议就得关闭RS485
    #if USE_RC
    pulse_init();
    #else
    rs485_init(115200);
	printf("power on \r\n");
    #endif
    delay_ms(50);
    
    
    g_bldc_motorA.pwm_duty = 0;
    g_bldc_motorA.run_flag = STOP;
    g_bldc_motorA.setdir = CW;
    g_bldc_motorA.max_c = RESET;
    g_bldc_motorA.max_t = RESET;
    g_bldc_motorA.low_p = RESET;
    g_bldc_motorA.hall_miss = RESET;
    g_bldc_motorA.v_bus = 20000;
    
    
    g_bldc_motorB.pwm_duty = 0;
    g_bldc_motorB.run_flag = STOP;
    g_bldc_motorB.setdir = CCW;
    g_bldc_motorB.max_c = RESET;
    g_bldc_motorB.max_t = RESET;
    g_bldc_motorB.low_p = RESET;
    g_bldc_motorB.hall_miss = RESET;
    g_bldc_motorB.v_bus = 20000;
    
    
    bldc_init(1000-1,HZ_P_RUN-1);
    delay_ms(50);
    
    adc1_dma_init();
    
    pid_init(100);
    
    
    g_iwdg_handle.Instance = IWDG;
    g_iwdg_handle.Init.Prescaler = IWDG_PRESCALER_64; /* 设置IWDG分频系数 40k*/
    g_iwdg_handle.Init.Reload = 125;     /* 重装载值 */
    HAL_IWDG_Init(&g_iwdg_handle);       /* 初始化IWDG并启动 */
    
    while(1)
    {
        HAL_IWDG_Refresh(&g_iwdg_handle);  /* 重装载计数器 */
        
        if( g_bldc_time.g_time_task1 >= 5 )
        {
            g_bldc_time.g_time_task1 = 0;
            #if USE_RC 
            /* RC通讯 */
            pulse_fetinst(0) ;
            #else
            /* wifi_now */
            ESP32_fetinst(1);
            #endif
        }
        
        if( g_bldc_time.g_time_task2 >= 20 )
        {
            g_bldc_time.g_time_task2 = 0;
            over_load(&g_bldc_motorA);
            over_load(&g_bldc_motorB);
        }
        
        if( g_bldc_time.g_time_task3 >= 5 )
        {
            uint16_t speed_temp = 0;
            g_bldc_time.g_time_task3 = 0;
            /* 计算转速 */
            
            /*RPM/MIN = 
            列子: PWM周期时间 = 18K = 0.055555ms ,计数值=10 极对数 = 7;
            RPM/min = (60000 / (10 * 2 * 0.0555)) / 7 = 7722rpm/min
            */
            
            //保证被除数不为0,同时过滤高频干扰
            if(g_bldc_motorA.step_all_time > 2)
            {
                speed_temp = 61714 / g_bldc_motorA.step_all_time; //7对级的rpm/min
                if(speed_temp < 10)
                {
                    g_bldc_motorA.speed = 0;
                }else
                {
                    g_bldc_motorA.speed = (speed_temp + 7 * g_bldc_motorA.speed) / 8;
                }
            }
            if(g_bldc_motorB.step_all_time > 2)
            {
                speed_temp = 61714 / g_bldc_motorB.step_all_time;
                if(speed_temp < 10)
                {
                    g_bldc_motorB.speed = 0;
                }else
                {
                    g_bldc_motorB.speed = (speed_temp + 7 * g_bldc_motorB.speed) / 8;
                }
            }
            
        }
        
        
        if( g_bldc_time.g_time_task4 >= 1 )
        {
            g_bldc_time.g_time_task4 = 0;
            /* 计算电流，温度*/
            adc_dma_conver();
        }
        
        
    }
}




void over_load(_bldc_obj *motor_temp)
{
    //低电压保护8s
    if(motor_temp->v_bus  <  27000)  
    {
        motor_temp->low_p = SET;
    }else if(motor_temp->v_bus  >  27500)
    {
        motor_temp->low_p = RESET;
    }
    
    //温度保护与释放
    if(motor_temp->v_t  >  HOT_OTP) 
    {
        motor_temp->max_t = SET;
    }else if(motor_temp->v_t < (HOT_OTP-2000))
    {
        motor_temp->max_t = RESET;
    }
    
    
    //过流保护与释放
    if(motor_temp->current > MAX_CURRENT)
    {
        motor_temp->max_c_count++;
        if(motor_temp->max_c_count > 100  &&  motor_temp->speed  < 60)
        {
            motor_temp->max_c_count = 200;
            motor_temp->locked_rotor = SET;
        }else if(motor_temp->max_c_count > 10)
        {
            motor_temp->max_c = SET;
        } 
    }else
    {
        motor_temp->max_c_count = 0;
        motor_temp->max_c = RESET;
        motor_temp->locked_rotor = RESET;
    }
}
