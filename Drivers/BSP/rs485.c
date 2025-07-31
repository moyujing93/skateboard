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

/******************************************************************************************/

//抛物线油门
const uint16_t parabola_table[100] = {
    0, 0, 0, 1, 2, 3, 4, 5,
    7, 8, 10, 12, 15, 17, 20, 23,
    26, 29, 33, 37, 41, 45, 49, 54,
    59, 64, 69, 74, 80, 86, 92, 98,
    104, 111, 118, 125, 132, 140, 147, 155,
    163, 172, 180, 189, 198, 207, 216, 225,
    235, 245, 255, 265, 276, 287, 298, 309,
    320, 331, 343, 355, 367, 380, 392, 405,
    418, 431, 444, 458, 472, 486, 500, 514,
    529, 544, 559, 574, 589, 605, 621, 637,
    653, 669, 686, 703, 720, 737, 755, 772,
    790, 808, 826, 845, 864, 882, 902, 921,
    940, 960, 980, 1000
};

//pid电流刹车
const uint16_t  Break_pidcc[20] =
{
    1000,
    1150,
    1320,
    1520,
    1750,
    2000,
    2300,
    2660,
    3060,
    3520,
    
    4050,
    4650,
    5350,
    6150,
    7080,
    8140,
    9360,
    11000,
    13300,
    16000
};

volatile rs485_typedef rs485_struct;
uint32_t esp32_lose_time = 0;

PID_TypeDef  rs_current_pid_temp[2] = { {0},{0} };         /* 电流环PID参数结构体 */

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
	MX_USART1_UART_Init();
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
    for (n = 0; n < BitNumber; n++) {/*此处的6 -- 要校验的位数为6个*/
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
 * @brief       刹车API
 * @param       电机结构体指针，
 * @retval      PID使用的值，不使用PID时电机结构体里直接对brake_duty进行赋值
 */
uint16_t motor_break( _bldc_obj *motor, int motor_control)
{
    uint16_t num;
    
    //设置标志
    motor->run_flag = BBK;
    
    //设置目标值
    num = int_limit(motor_control,0,500);
    num = 1 + (num  / 25);
    num = int_limit(num,1,20);
    num = 2 * Break_pidcc[num - 1];
    
    return num;
}


/**
 * @brief       处理主机的请求
 * @param       0=转速控制  1=电流控制  2=占空比控制
 * @retval      
 */
void ESP32_fetinst(uint8_t mode)
{
    static uint16_t speed_L,speed_R;
    struct_read esp_control = {0};
    static int motor_control = 0;
    uint16_t crc_temp = 0;
    
    
    static uint32_t tim_test = 0;
    tim_test++;  //test
    
    usart_receive_data();
    if(g_esp32_sta == 1)    /* 总线有消息 */
    {
        //接收消息
        esp_control = g_esp32_struct;
        g_esp32_sta = 0;
        
        crc_temp = esp_control.crc;
        esp_control.crc = 0;
        esp_control.crc = ModbusCRC16_Check((uint8_t*)&esp_control,sizeof(esp_control));
        if(esp_control.crc != crc_temp)  return;
        
        esp32_lose_time = g_bldc_time.g_time_sys;
        motor_control = esp_control.adc - 1500;
        motor_control = int_limit(motor_control,-500,500);
        if(mode == 0)      //转速控制
        {
            
        }else if(mode == 1)    //电流控制
        {
            
            g_MA_speed_pid.SetPoint  = MAX_RPM;
            g_MB_speed_pid.SetPoint  = MAX_RPM;
            
            //正转，反转立即切换
            if(esp_control.dir)
            {
                g_bldc_motorA.setdir = CCW;
                g_bldc_motorB.setdir = CW;
                
            }else
            {
                g_bldc_motorA.setdir = CW;
                g_bldc_motorB.setdir = CCW;
                
            }
            
            //设置参数
            if(motor_control > 30)  // 加速
            {
                uint8_t run_break_sta = 0;
                
                //    电机A   注意滑板电机是左右方向相反的
                if(g_bldc_motorA.setdir  != g_bldc_motorA.step_dir && g_bldc_motorA.speed > 500)
                {
                    //采用制动
                    run_break_sta = 1;
                }
                //    电机B   注意滑板电机是左右方向相反的
                if(g_bldc_motorB.setdir  != g_bldc_motorB.step_dir && g_bldc_motorB.speed > 500)
                {
                    //采用制动
                    run_break_sta = 1;
                }
                
                //根据状态执行操作
                if(run_break_sta == 1)
                {
                    g_MA_break_pid.SetPoint = motor_break(&g_bldc_motorA,500);
                    g_MB_break_pid.SetPoint = motor_break(&g_bldc_motorB,500);
                }else
                {
                    uint16_t  cun_temp = (motor_control * SET_CURRENT) / 500;
                    g_bldc_motorA.run_flag = RUN;
                    rs_current_pid_temp[0].SetPoint  = cun_temp;
                    g_bldc_motorB.run_flag = RUN;
                    rs_current_pid_temp[1].SetPoint  = cun_temp;
                }
                
            }else if(motor_control < -30)  // 刹车
            {
                g_MA_break_pid.SetPoint = motor_break(&g_bldc_motorA,-motor_control);
                g_MB_break_pid.SetPoint = motor_break(&g_bldc_motorB,-motor_control);
                
            }else
            {
                g_bldc_motorA.run_flag = STOP;
                g_bldc_motorB.run_flag = STOP;
                g_MA_break_pid.SetPoint = 0;
                g_MB_break_pid.SetPoint = 0;
                rs_current_pid_temp[0].SetPoint = 0;
                rs_current_pid_temp[1].SetPoint = 0;
            }
            
        }else if(mode == 2)    //占空比控制
        {
            
        }
        
        //向ESP32发送消息
        
        g_uart_send_esp.ff = 0x7b;
        g_uart_send_esp.bb = 0x7d;
        
        g_uart_send_esp.v_bus = g_bldc_motorA.v_bus;
        
        if(tim_test > 6000)
        {
            tim_test = 6000;
            g_uart_send_esp.v_tee = g_bldc_motorA.v_t / 100;
        }else
        {
            g_uart_send_esp.v_tee = TEST_SYS_TICK / 72;  //test
        }
        //提示异常
        if(g_bldc_motorA.hall_miss == SET)
        {
            g_uart_send_esp.v_tee = 0xffe1;
        }else if(g_bldc_motorA.max_c == 1)
        {
            g_uart_send_esp.v_tee = 0xffe2;
        }else if(g_bldc_motorA.max_t == 1)
        {
            g_uart_send_esp.v_tee = 0xffe3;
        }else if(g_bldc_motorA.locked_rotor == 1)
        {
            g_uart_send_esp.v_tee = 0xffe4;
        }
        
        
        //取最小的速度，避免打滑
        speed_L = g_bldc_motorA.speed;
        speed_R = g_bldc_motorB.speed;
        
        //有一个电机没接
        if(speed_L == 0 || speed_R == 0)
        {
            if(speed_L == 0)
            {
                g_uart_send_esp.speed = speed_R;
            }else
            {
                g_uart_send_esp.speed = speed_L;
            }
        }else
        {
            if(speed_L > speed_R)
            {
                g_uart_send_esp.speed = speed_R;
            }else
            {
                g_uart_send_esp.speed = speed_L;
            }
        }
        //电流
        g_uart_send_esp.current = (g_bldc_motorA.current + g_bldc_motorB.current) / 2;
        //crc
        
        //发送信息
        uint32_t  data_size = sizeof(g_uart_send_esp);
        g_uart_send_esp.crc = 0;
        g_uart_send_esp.crc = ModbusCRC16_Check((uint8_t*)&g_uart_send_esp,data_size);
        
        uint8_t data_temp[data_size];
        memcpy(&data_temp[0],&g_uart_send_esp,data_size);
        
        usart_send_data(&data_temp[0],data_size);
        
    }else
    {
        if((g_bldc_time.g_time_sys - esp32_lose_time) > 200)
        {
            g_bldc_motorA.run_flag = STOP;
            g_bldc_motorB.run_flag = STOP;
        }
    }
}
