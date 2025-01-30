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
#ifndef  __BLDC_H
#define  __BLDC_H

#include "./SYSTEM/sys/sys.h"


/*********************电机A*************************/

//霍尔
#define MA_HALL_U_PIN           GPIO_PIN_5     /* U */
#define MA_HALL_U_PORT          GPIOB

#define MA_HALL_V_PIN           GPIO_PIN_4     /* V */
#define MA_HALL_V_PORT          GPIOB

#define MA_HALL_W_PIN           GPIO_PIN_3     /* W */
#define MA_HALL_W_PORT          GPIOB

//上桥臂IO定义
#define MA_UH_PWM_PIN             GPIO_PIN_6
#define MA_UH_PWM_PORT            GPIOC

#define MA_VH_PWM_PIN             GPIO_PIN_7
#define MA_VH_PWM_PORT            GPIOC

#define MA_WH_PWM_PIN             GPIO_PIN_8
#define MA_WH_PWM_PORT            GPIOC


//下桥臂IO定义
#define MA_UL_PIN             GPIO_PIN_7
#define MA_UL_PORT            GPIOA

#define MA_VL_PIN             GPIO_PIN_0
#define MA_VL_PORT            GPIOB

#define MA_WL_PIN             GPIO_PIN_1
#define MA_WL_PORT            GPIOB


/*********************电机B*************************/

//霍尔
#define MB_HALL_U_PIN           GPIO_PIN_12     /* U */
#define MB_HALL_U_PORT          GPIOC

#define MB_HALL_V_PIN           GPIO_PIN_11     /* V */
#define MB_HALL_V_PORT          GPIOC

#define MB_HALL_W_PIN           GPIO_PIN_10     /* W */
#define MB_HALL_W_PORT          GPIOC

//上桥臂IO定义
#define MB_UH_PWM_PIN             GPIO_PIN_8
#define MB_UH_PWM_PORT            GPIOA

#define MB_VH_PWM_PIN             GPIO_PIN_9
#define MB_VH_PWM_PORT            GPIOA

#define MB_WH_PWM_PIN             GPIO_PIN_10
#define MB_WH_PWM_PORT            GPIOA

//下桥臂IO定义
#define MB_UL_PIN             GPIO_PIN_13
#define MB_UL_PORT            GPIOB

#define MB_VL_PIN             GPIO_PIN_14
#define MB_VL_PORT            GPIOB

#define MB_WL_PIN             GPIO_PIN_15
#define MB_WL_PORT            GPIOB


/********************* 电源开关 *************************/

#define MX_POWER_PIN             GPIO_PIN_12
#define MX_POWER_PORT            GPIOA




/*************************************** 电机控制状态 ***********************************************/
#define CCW                         (1)                 /* 逆时针 */
#define CW                          (0)                 /* 顺时针 */
#define RUN                         (1)                 /* 电机运动标志 */
#define STOP                        (0)                 /* 电机停机标志 */
#define HOT_OTP                     (10000)             /* 高温保护阈值25度=2500 */
#define MAX_CURRENT                 (50000)             /* 能检测的最大电流ma */
#define SET_CURRENT                 (30000)             /* 能控制的最大电流ma */
#define MAX_PWM_SET                  (900)              /* 全局最大加速PWM */
#define MAX_PWM_BRAKE_SET            (990)              /* 全局最大刹车PWM */
#define MAX_RPM                     (8000)              /* 最大转速RPM/MIN */
#define BK_LEVEL                    (5)                 /* 电流锁止刹车占空比的除数 */
#define HZ_P_RUN                    (5)                 /* tim的plc分频 */
#define HZ_P_BK                     (10)                /* tim的plc分频 */

/***************************************** 函数 *************************************************/
typedef enum
{
    MOTORA  =   1,
    MOTORB  =   2
}motornum_Type;



typedef void(*pctr) (void);             /* 六部换相函数指针 */

typedef struct
{
    volatile uint32_t   g_time_sys;
    volatile uint32_t   g_time_task1;
    volatile uint32_t   g_time_task2;
    volatile uint32_t   g_time_task3;
    volatile uint32_t   g_time_task4;
}_gtime_obj;


typedef struct
{
    volatile uint8_t    run_flag;       /* 运行标志 */
    volatile uint8_t    locked_rotor;   /* 堵转标记 */
    volatile uint8_t    max_c;   /* 过流保护 */
    volatile uint8_t    max_c_count;   /* 过流时间计数 */
    volatile uint8_t    max_t;   /* 过热保护 */
    volatile uint8_t    low_p;   /* 低压保护 */
    volatile uint8_t    low_p_count;   /* 时间计数 */
    
    volatile uint8_t    hall_erro_count;  /* 霍尔健康度 */
    volatile uint8_t    hall_miss;        /* 霍尔丢失 */
    volatile uint8_t    step_sta;         /* 本次霍尔状态 */
    volatile uint8_t    step_last;        /* 上次霍尔状态 */
    volatile uint8_t    step_dir;         /* 用于计算旋转方向 */
    volatile uint16_t   step_all_time;    /* 3步换向用时，计算用 */
    volatile uint16_t   step_count;       /* 用于计算旋转方向 */
    
    
    volatile uint8_t    setdir;            /* 电机目标旋转方向 */
    volatile uint16_t        speed;          /* 电机速度 RMP/min */
//    volatile int32_t    pos;            /* 电机位置 */
    volatile uint16_t   current;        /* 电机电流 mA  */
    
    
    volatile uint16_t   v_bus;          /* 主线电压 mv */
    volatile uint16_t   v_t;         /* 驱动板温度 放大100倍 125 = 1.25摄氏度 */
    volatile uint16_t   brake_flag;     /* 刹车标志 0 = 无刹车 1 = 电阻刹车 2 = 电流刹车 */
    volatile uint16_t   pwm_duty;       /* 电机占空比 */
    volatile uint16_t   brake_duty;     /* 刹车时的占空比 */
    
    
//    volatile uint32_t   hall_keep_t;    /* 霍尔保持时间 */
//    volatile uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
//    volatile uint32_t   lock_time;      /* 电机堵转时间 */
//    volatile uint32_t   lock_time;      /* 电机堵转时间 */
//    volatile uint32_t   no_single;
//    volatile uint32_t   count_j;
}_bldc_obj;


extern TIM_HandleTypeDef   g_MA_timx_handle;
extern TIM_HandleTypeDef   g_MB_timx_handle;

extern _bldc_obj g_bldc_motorA;
extern _bldc_obj g_bldc_motorB;
extern _gtime_obj  g_bldc_time;

extern uint16_t MAX_PWM;
extern uint16_t MAX_PWM_BRAKE;

void bldc_init(uint16_t arr, uint16_t psc);

int int_abs(int num);
int int_limit(int num,int min ,int max);



#endif
