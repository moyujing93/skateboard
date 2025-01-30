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

#define  control_diff   50   //油门零点位置飘移



//占空比刹车
uint16_t  Break_num[20] =
{
    0,
    0,
    50,
    100,
    150,
    200,
    250,
    300,
    350,
    400,
    450,
    500,
    550,
    600,
    650,
    700,
    750,
    800,
    850,
    900
};

//pid电流刹车
uint16_t  Break_pidcc[20] =
{
    500,
    1225,
    1950,
    2675,
    3400,
    4125,
    4850,
    5575,
    6300,
    7025,
    7750,
    8475,
    9200,
    9925,
    10650,
    11370,
    12100,
    12800,
    13500,
    14250
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
 * @brief       刹车API
 * @param       
 * @retval      
 */
static void motor_break(int motor_control)
{
    g_bldc_motorA.run_flag = STOP;
    g_bldc_motorB.run_flag = STOP;

    g_bldc_motorA.brake_flag = 1;
    g_bldc_motorB.brake_flag = 1;

    if(BK_UES_PID == 1)
    {
        uint8_t temp = ((abs(motor_control) - control_diff - 1) * 20) / (500 - control_diff);
        temp = int_limit(temp,1,20);
        g_MA_break_pid.SetPoint = 1.2f * Break_pidcc[temp - 1];
        g_MB_break_pid.SetPoint = g_MA_break_pid.SetPoint;
    //    MX_brake_duty   =  Break_num[((abs(motor_control) - control_diff - 1) * 8) / (500 - control_diff)];
    }else
    {
        motor_control = int_abs(motor_control);
        motor_control = int_limit(motor_control,0,500);
        motor_control = 100 + (abs(motor_control) * 1.8f);
        motor_control = int_limit(motor_control,0,980);
        
        g_bldc_motorA.brake_duty = motor_control;
        g_bldc_motorB.brake_duty = motor_control;
        
    }
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
    float  cun_temp;
    if(g_esp32_struct.sta == 1)    /* 总线有消息 */
    {
        
        esp32_lose_time = g_bldc_time.g_time_sys;
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
                g_bldc_motorA.run_flag = RUN;
                g_bldc_motorB.run_flag = RUN;
                
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorB.brake_flag = 0;
                
                
                //注意滑板电机是左右方向相反的
                
                cun_temp = (abs(motor_control) * SET_CURRENT) / 500;
                
                if(g_bldc_motorA.setdir  != g_bldc_motorA.step_dir)
                {
                    //电机在受外力反转，限制电流
                    g_MA_current_pid.SetPoint = int_limit(cun_temp,0,MAX_CURRENT / 3);
                }else
                {
                    g_MA_current_pid.SetPoint  = cun_temp;
                }
                
                
                if( g_bldc_motorB.setdir != g_bldc_motorB.step_dir)
                {
                    //电机在受外力反转，限制电流
                    g_MB_current_pid.SetPoint = int_limit(cun_temp,0,MAX_CURRENT / 3);
                    
                }else
                {
                    g_MB_current_pid.SetPoint  = cun_temp;
                }
                
            }else if(motor_control < -(control_diff*1.5f))  // 刹车
            {
                
                motor_break(motor_control);
                
            }else
            {
                g_bldc_motorA.run_flag = STOP;
                g_bldc_motorB.run_flag = STOP;
                
                g_bldc_motorA.brake_flag = 0;
                g_bldc_motorB.brake_flag = 0;
                
                //正转，反转，停止，只有在转速为0时才能切换
                if(esp_control.dir)
                {
                    g_bldc_motorA.setdir = CCW;
                    g_bldc_motorB.setdir = CW;
                    
                }else
                {
                    g_bldc_motorA.setdir = CW;
                    g_bldc_motorB.setdir = CCW;
                    
                }
                
            }
            
        }else if(mode == 2)    //占空比控制
        {
            
        }
        
        //向ESP32发送消息
        
        g_uart_send_esp.ff = 0x7b;
        g_uart_send_esp.bb = 0x7d;
        
        g_uart_send_esp.v_bus = g_bldc_motorA.v_bus;
        g_uart_send_esp.v_bus = 19000 + (6 * g_bldc_motorA.pwm_duty);//19000;//25000;
        g_uart_send_esp.v_bus = 19000 + (1.0f * g_bldc_motorA.current);//19000;//25000;
        g_uart_send_esp.v_tee = g_bldc_motorA.v_t / 100;
        //调试用。。。。。
        if(g_bldc_motorA.hall_miss == SET)
        {
            g_uart_send_esp.v_tee = 1;
        }else if(g_bldc_motorA.max_c == 1)
        {
            g_uart_send_esp.v_tee = 2;
        }else if(g_bldc_motorA.max_t == 1)
        {
            g_uart_send_esp.v_tee = 3;
        }else if(g_bldc_motorA.locked_rotor == 1)
        {
            g_uart_send_esp.v_tee = 4;
        }else
        {
//            g_uart_send_esp.v_tee = g_bldc_motorB.step_all_time;
            g_uart_send_esp.v_tee = g_bldc_motorA.v_bus / 1000;
        }
        
        
        //速度计算
        //取最小的速度，避免打滑
        //电机齿 14T  车轮齿 36T  车轮直径 90mm  90mm * 3.14 * (rpm/min / (36t/14t)) * 60min = mm/h    0.0065 * rpm
        speed_L = g_bldc_motorA.speed * 0.0073f;
        speed_R = g_bldc_motorB.speed * 0.0073f;
        
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
        //发送信息
        usart_send_data((uint8_t*)&g_uart_send_esp,sizeof(g_uart_send_esp));
        
    }else
    {
        if((g_bldc_time.g_time_sys - esp32_lose_time) > 1000)
        {
            g_bldc_motorA.run_flag = STOP;
            g_bldc_motorB.run_flag = STOP;
            
            g_bldc_motorA.brake_flag = 1;
            g_bldc_motorB.brake_flag = 1;
            
            if(BK_UES_PID == 1)
            {
                g_MA_break_pid.SetPoint = Break_pidcc[2];
                g_MB_break_pid.SetPoint = Break_pidcc[2];
            }else
            {
                g_bldc_motorA.brake_duty = Break_num[8];
                g_bldc_motorB.brake_duty = Break_num[8];
            }
        }
    }
}
