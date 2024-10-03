/**
 ****************************************************************************************************
 * @file        rs485.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       RS485 ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200424
 * ��һ�η���
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
 * @brief       RS485��ʼ������
 *   @note      �ú�����Ҫ�ǳ�ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @retval      ��
 */
void rs485_init(uint32_t baudrate)
{
    /* ����usart��ʼ������ */
    usart_init(baudrate);
}
/**
 * @brief       CRC����
 * @param       �������׵�ַ
 * @param       ���͵��ֽ���
 * @retval      ��
 */
unsigned short ModbusCRC16_Check(uint8_t *array, int BitNumber)
{
    unsigned short tmp = 0xffff;
    
    int n = 0, i = 0;
    for (n = 0; n < (BitNumber - 2); n++) {/*�˴���6 -- ҪУ���λ��Ϊ6��*/
        tmp = array[n] ^ tmp;
        for (i = 0; i < 8; i++) {  /*�˴���8 -- ָÿһ��char������8bit��ÿbit��Ҫ����*/
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
 * @brief       ���յ�������
 * @param       buf     : ���ջ������׵�ַ
 * @retval      ���ؽ��ܵ����ֽ���,Ϊ0���ǽ�������ʧ��
 */
uint8_t rs485_receive_data(uint8_t *buf, uint16_t buf_size)
{
    uint8_t sta = 0;
    sta = usart_receive_data(buf,buf_size);
    return sta;
}

/**
 * @brief       ��������������
 * @param       0=ת�ٿ���  1=��������  2=ռ�ձȿ���
 * @retval      
 */
void ESP32_fetinst(uint8_t mode)
{
    struct_send esp_control = {0};
    static int motor_control = 0;
    
    if(g_esp32_struct.sta == 1)    /* ��������Ϣ */
    {
        esp_control = g_esp32_struct;
        g_esp32_struct.sta = 0;
        motor_control = esp_control.adc - 1500;
        motor_control = int_limit(motor_control,-500,500);
        if(mode == 0)      //ת�ٿ���
        {
            pid_sta = 1;
            g_MA_current_pid.SetPoint  = MAX_CURRENT / 3;
            g_MB_current_pid.SetPoint  = MAX_CURRENT / 3;
            
            //��ת����ת��ֹͣ
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //���ò���
            if(motor_control > 50)  // ����
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
            
        }else if(mode == 1)    //��������
        {
            pid_sta = 1;
            g_MA_speed_pid.SetPoint  = MAX_RPM;
            g_MB_speed_pid.SetPoint  = MAX_RPM;
            
            //��ת����ת��ֹͣ
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //���ò���
            if(motor_control > 50)  // ����
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
            
        }else if(mode == 2)    //ռ�ձȿ���
        {
            pid_sta = 0;
            //���PIDĿ��ֵ
            g_MA_speed_pid.SetPoint  = 0;
            g_MA_current_pid.SetPoint  = 0;
            g_MB_speed_pid.SetPoint  = 0;
            g_MB_current_pid.SetPoint  = 0;
            
            //��ת����ת��ֹͣ
            if(esp_control.dir)
            {
                g_bldc_motorA.dir = CCW;
                g_bldc_motorB.dir = CW;
                
            }else
            {
                g_bldc_motorA.dir = CW;
                g_bldc_motorB.dir = CCW;
                
            }
            //���ò���
            if(motor_control > 50)  // ����
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
