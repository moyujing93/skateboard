/**
 ****************************************************************************************************
 * @file        rs485.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       RS485 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200424
 * 第一次发布
 *
 ****************************************************************************************************
 */


#include <stdlib.h>
#include <string.h>
#include "./BSP/rs485.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/bldc.h"
#include "./BSP/pid.h"

/******************************************************************************************/

uint8_t  Break_num[8] =
{
    1,
    2,
    3,
    5,
    7,
    9,
    12,
    15
};
volatile rs485_typedef rs485_struct;
/******************************************************************************************/

/**
 * @brief       RS485初始化函数
 *   @note      该函数主要是初始化串口
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void rs485_init(uint32_t baudrate)
{
    /* 调用usart初始化函数 */
    usart_init(baudrate);
}
/**
 * @brief       CRC检验
 * @param       发送区首地址
 * @param       发送的字节数
 * @retval      无
 */
unsigned short ModbusCRC16_Check(uint8_t *array, int BitNumber)
{
    unsigned short tmp = 0xffff;
    
    int n = 0, i = 0;
    for (n = 0; n < (BitNumber - 2); n++) {/*此处的6 -- 要校验的位数为6个*/
        tmp = array[n] ^ tmp;
        for (i = 0; i < 8; i++) {  /*此处的8 -- 指每一个char类型又8bit，每bit都要处理*/
            if (tmp & 0x01) {
                tmp = tmp >> 1;
                tmp = tmp ^ 0xa001;
            } else {
                tmp = tmp >> 1;
            }
        }
    }
    return tmp;
}


/**
 * @brief       接收到的数据
 * @param       buf     : 接收缓冲区首地址
 * @retval      返回接受到的字节数,为0则是接受数据失败
 */
uint8_t rs485_receive_data(uint8_t *buf, uint16_t buf_size)
{
    uint8_t sta = 0;
    sta = usart_receive_data(buf,buf_size);
    return sta;
}

/**
 * @brief       处理主机的请求
 * @param       0=转速控制  1=电流控制  2=占空比控制
 * @retval      
 */
void ESP32_fetinst(uint8_t mode)
{
    struct_send esp_control = {0};
    static int motor_control = 0;
    
    if(g_esp32_struct.sta == 1)    /* 总线有消息 */
    {
        esp_control = g_esp32_struct;
        g_esp32_struct.sta = 0;
        motor_control = esp_control.adc - 1500;
        motor_control = int_limit(motor_control,-500,500);
        if(mode == 0)      //转速控制
        {
            pid_sta = 1;
            g_MA_current_pid.SetPoint  = MAX_CURRENT / 3;
            g_MB_current_pid.SetPoint  = MAX_CURRENT / 3;
            
            //正转，反转，停止
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //设置参数
            if(motor_control > 50)  // 加速
            {
                g_bldc_motorA.run_flag = RUN;
                g_bldc_motorB.run_flag = RUN;
                g_MA_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
                g_MB_speed_pid.SetPoint  = (abs(motor_control) * MAX_RPM) / 500;
            }else if(motor_control < -50)
            {
                g_bldc_motorA.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                
                g_bldc_motorB.run_flag = STOP;
                g_MB_speed_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;
                
                g_bldc_motorA.brake_flag = 1;
                g_bldc_motorB.brake_flag = 1;
                g_bldc_motorA.brake_duty = (abs(motor_control) * MAX_BREAK) / 500;
                g_bldc_motorB.brake_duty = (abs(motor_control) * MAX_BREAK) / 500;
            }else if(motor_control > -30 && motor_control < 30)
            {
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorA.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                
                g_bldc_motorB.brake_flag = 0;
                g_bldc_motorB.run_flag = STOP;
                g_MB_speed_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;
                
            }
            
        }else if(mode == 1)    //电流控制
        {
            pid_sta = 1;
            g_MA_speed_pid.SetPoint  = MAX_RPM;
            g_MB_speed_pid.SetPoint  = MAX_RPM;
            
            //正转，反转，停止
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //设置参数
            if(motor_control > 50)  // 加速
            {
                g_bldc_motorA.run_flag = RUN;
                g_bldc_motorB.run_flag = RUN;
                g_MA_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
                g_MB_current_pid.SetPoint  = (abs(motor_control) * MAX_CURRENT) / 500;
            }else if(motor_control < -50)
            {
                g_bldc_motorA.brake_flag = 1;
                g_bldc_motorA.brake_duty = (abs(motor_control) * MAX_BREAK) / 500;
                g_bldc_motorA.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                
                g_bldc_motorB.brake_flag = 1;
                g_bldc_motorB.brake_duty = (abs(motor_control) * MAX_BREAK) / 500;
                g_bldc_motorB.run_flag = STOP;
                g_MB_speed_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;
                
            }else if(motor_control > -30 && motor_control < 30)
            {
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorA.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                
                g_bldc_motorB.brake_flag = 0;
                g_bldc_motorB.run_flag = STOP;
                g_MB_speed_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;

            }
            
        }else if(mode == 2)    //占空比控制
        {
            pid_sta = 0;
            //清空PID目标值
            g_MA_speed_pid.SetPoint  = 0;
            g_MA_current_pid.SetPoint  = 0;
            g_MB_speed_pid.SetPoint  = 0;
            g_MB_current_pid.SetPoint  = 0;
            
            //正转，反转，停止
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //设置参数
            if(motor_control > 50)  // 加速
            {
                g_bldc_motorA.run_flag = RUN;
                g_bldc_motorA.pwm_duty = (abs(motor_control) * MAX_PWM) / 500;
                
                g_bldc_motorB.run_flag = RUN;
                g_bldc_motorB.pwm_duty = (abs(motor_control) * MAX_PWM) / 500;
            }else if(motor_control < -50)
            {
                g_bldc_motorA.brake_flag = 1;
                g_bldc_motorA.pwm_duty = 0;
                g_bldc_motorA.brake_duty = Break_num[((abs(motor_control) - 51) * 8) / 450];
                g_bldc_motorA.run_flag = STOP;
                
                g_bldc_motorB.brake_flag = 1;
                g_bldc_motorB.pwm_duty = 0;
                g_bldc_motorB.brake_duty = g_bldc_motorA.brake_duty;
                g_bldc_motorB.run_flag = STOP;
                
            }else if(motor_control > -30 && motor_control < 30)
            {
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorA.run_flag = STOP;
                g_bldc_motorA.pwm_duty = 0;
                
                g_bldc_motorB.brake_flag = 0;
                g_bldc_motorB.run_flag = STOP;
                g_bldc_motorB.pwm_duty = 0;
                
            }
            
        }
    }else
    {
        if((time_num - esp32_lose_time) > 1000)
        {
            g_bldc_motorA.brake_flag = 0;
            g_bldc_motorA.run_flag = STOP;
            g_MA_speed_pid.SetPoint  = 0;
            g_MA_current_pid.SetPoint  = 0;
            
            g_bldc_motorB.brake_flag = 0;
            g_bldc_motorB.run_flag = STOP;
            g_MB_speed_pid.SetPoint  = 0;
            g_MB_current_pid.SetPoint  = 0;
        }
    }
}
