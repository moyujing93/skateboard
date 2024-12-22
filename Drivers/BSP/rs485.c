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

//占空比刹车
uint16_t  Break_num[8] =
{
    200,
    250,
    300,
    350,
    450,
    550,
    650,
    750
};

//pid电流刹车
uint16_t  Break_pidcc[8] =
{
    500,
    1000,
    1500,
    2000,
    3000,
    4000,
    5000,
    6000
};

volatile rs485_typedef rs485_struct;
uint32_t esp32_lose_time = 0;

struct_uart_send  g_uart_send_esp = {0};
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
#define  control_diff   35
void ESP32_fetinst(uint8_t mode)
{
    struct_read esp_control = {0};
    static int motor_control = 0;
    
    if(g_esp32_struct.sta == 1)    /* 总线有消息 */
    {
        
        esp32_lose_time = time_num;
        //接收消息
        esp_control = g_esp32_struct;
        g_esp32_struct.sta = 0;
        motor_control = esp_control.adc - 1500;
        motor_control = int_limit(motor_control,-500,500);
        if(mode == 0)      //转速控制
        {
            
        }else if(mode == 1)    //电流控制
        {
            
            g_MA_speed_pid.SetPoint  = MAX_RPM;
            g_MB_speed_pid.SetPoint  = MAX_RPM;
            
            //设置参数
            if(motor_control > control_diff)  // 加速
            {
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorB.brake_flag = 0;
                
                g_bldc_motorA.run_flag = RUN;
                g_bldc_motorB.run_flag = RUN;
                
                g_MA_current_pid.SetPoint  = (abs(motor_control) * SET_CURRENT) / 500;
                g_MB_current_pid.SetPoint  = g_MA_current_pid.SetPoint;
                
            }else if(motor_control < -control_diff)  // 刹车
            {
                g_bldc_motorA.run_flag = STOP;
                g_bldc_motorB.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MB_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;
                g_bldc_motorA.brake_flag = 1;
                g_bldc_motorB.brake_flag = 1;
                g_bldc_motorA.pwm_duty = 0;
                g_bldc_motorB.pwm_duty = 0;
                
                if(BK_UES_PID == 1)
                {
                    g_MX_break_pid.SetPoint = Break_pidcc[((abs(motor_control) - control_diff - 1) * 8) / (500 - control_diff)];
                }else
                {
                    g_bldc_motorA.brake_duty = Break_num[((abs(motor_control) - control_diff - 1) * 8) / (500 - control_diff)];
                    g_bldc_motorB.brake_duty = g_bldc_motorA.brake_duty;
                }
                
                
                
            }else
            {
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorB.brake_flag = 0;
                g_bldc_motorA.run_flag = STOP;
                g_bldc_motorB.run_flag = STOP;
                g_MA_speed_pid.SetPoint  = 0;
                g_MB_speed_pid.SetPoint  = 0;
                g_MA_current_pid.SetPoint  = 0;
                g_MB_current_pid.SetPoint  = 0;
                
                
                //正转，反转，停止，只有在转速为0时才能切换
                if(esp_control.dir)
                {
                    g_bldc_motorA.dir = CCW;
                    g_bldc_motorB.dir = CW;
                    
                }else
                {
                    g_bldc_motorA.dir = CW;
                    g_bldc_motorB.dir = CCW;
                    
                }
                
            }
            
        }else if(mode == 2)    //占空比控制
        {
            
        }
        
        //向ESP32发送消息
        
        g_uart_send_esp.ff = 0x7b;
        g_uart_send_esp.bb = 0x7d;
        
        g_uart_send_esp.v_bus = g_bldc_motorA.v_bus;
        g_uart_send_esp.v_tee = g_bldc_motorA.v_t / 100;
        //取最小的速度，避免打滑
        //电机齿 14T  车轮齿 36T  车轮直径 90mm  90mm * 3.14 * (rpm/min / (36t/14t)) * 60min = mm/h    0.0065 * rpm
        g_uart_send_esp.speed = ((g_bldc_motorA.speed + g_bldc_motorB.speed) / 2 * 0.0065f); // (g_bldc_motorA.speed * 0.0065f) ;
//        g_uart_send_esp.speed = g_bldc_motorA.speed > g_bldc_motorB.speed ?  (g_bldc_motorB.speed * 0.0065f) : (g_bldc_motorA.speed * 0.0065f) ;
        g_uart_send_esp.current = (g_bldc_motorA.current + g_bldc_motorB.current) / 2;
        
        usart_send_data((uint8_t*)&g_uart_send_esp,sizeof(g_uart_send_esp));
        
    }else
    {
        if(int_abs(time_num - esp32_lose_time) > 1000)
        {
            g_bldc_motorA.brake_flag = 1;
            g_bldc_motorA.brake_duty = Break_num[2];
            g_bldc_motorA.run_flag = STOP;
            g_MA_speed_pid.SetPoint  = 0;
            g_MA_current_pid.SetPoint  = 0;
            
            g_bldc_motorB.brake_flag = 1;
            g_bldc_motorB.brake_duty = Break_num[2];
            g_bldc_motorB.run_flag = STOP;
            g_MB_speed_pid.SetPoint  = 0;
            g_MB_current_pid.SetPoint  = 0;
        }
    }
}
