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
#ifndef  __BLDC_H
#define  __BLDC_H

#include "./SYSTEM/sys/sys.h"


/*********************���A*************************/

//����
#define MA_HALL_U_PIN           GPIO_PIN_5     /* U */
#define MA_HALL_U_PORT          GPIOB

#define MA_HALL_V_PIN           GPIO_PIN_4     /* V */
#define MA_HALL_V_PORT          GPIOB

#define MA_HALL_W_PIN           GPIO_PIN_3     /* W */
#define MA_HALL_W_PORT          GPIOB

//���ű�IO����
#define MA_UH_PWM_PIN             GPIO_PIN_6
#define MA_UH_PWM_PORT            GPIOC

#define MA_VH_PWM_PIN             GPIO_PIN_7
#define MA_VH_PWM_PORT            GPIOC

#define MA_WH_PWM_PIN             GPIO_PIN_8
#define MA_WH_PWM_PORT            GPIOC


//���ű�IO����
#define MA_UL_PIN             GPIO_PIN_7
#define MA_UL_PORT            GPIOA

#define MA_VL_PIN             GPIO_PIN_0
#define MA_VL_PORT            GPIOB

#define MA_WL_PIN             GPIO_PIN_1
#define MA_WL_PORT            GPIOB


/*********************���B*************************/

//����
#define MB_HALL_U_PIN           GPIO_PIN_12     /* U */
#define MB_HALL_U_PORT          GPIOC

#define MB_HALL_V_PIN           GPIO_PIN_11     /* V */
#define MB_HALL_V_PORT          GPIOC

#define MB_HALL_W_PIN           GPIO_PIN_10     /* W */
#define MB_HALL_W_PORT          GPIOC

//���ű�IO����
#define MB_UH_PWM_PIN             GPIO_PIN_8
#define MB_UH_PWM_PORT            GPIOA

#define MB_VH_PWM_PIN             GPIO_PIN_9
#define MB_VH_PWM_PORT            GPIOA

#define MB_WH_PWM_PIN             GPIO_PIN_10
#define MB_WH_PWM_PORT            GPIOA

//���ű�IO����
#define MB_UL_PIN             GPIO_PIN_13
#define MB_UL_PORT            GPIOB

#define MB_VL_PIN             GPIO_PIN_14
#define MB_VL_PORT            GPIOB

#define MB_WL_PIN             GPIO_PIN_15
#define MB_WL_PORT            GPIOB


/*************************************** �������״̬ ***********************************************/
#define CCW                         (1)                 /* ��ʱ�� */
#define CW                          (2)                 /* ˳ʱ�� */
#define HALL_ERROR                  (0xF0)              /* ���������־ */
#define RUN                         (1)                 /* ����˶���־ */
#define STOP                        (0)                 /* ���ͣ����־ */
#define MAX_PWM                     (300)               /* ���ռ�ձ�0-1000 */
#define HOT_OTP                     (9000)             /* ���±�����ֵ25��=2500 */
#define MAX_CURRENT                 (15000)             /* ������ma */
#define MAX_RPM                     (2500)             /* ���ת��RPM/MIN */
#define MAX_BREAK                   (40)              /* ���ɲ��ռ�ձ� % */

/***************************************** ���� *************************************************/
typedef enum
{
    MOTORA  =   1,
    MOTORB  =   2
}motornum_Type;



typedef void(*pctr) (void);             /* �������ຯ��ָ�� */

typedef struct
{
    volatile uint8_t    run_flag;       /* ���б�־ */
    volatile uint8_t    locked_rotor;   /* ��ת��� */
    volatile uint8_t    max_c;   /* �������� */
    volatile uint8_t    max_t;   /* ���ȱ��� */
    volatile uint8_t    step_sta;       /* ���λ���״̬ */
    volatile uint32_t    hall_speed_num;/* ����ת�ٵļ���ֵ */
//    volatile uint8_t    step_last;      /* �ϴλ���״̬ */
    volatile uint8_t    dir;            /* �����ת���� */
//    volatile int32_t    pos;            /* ���λ�� */
    volatile uint16_t    speed;          /* ����ٶ� RMP/min */
    volatile uint16_t   current;        /* ������� mA  */
    volatile uint16_t   pwm_duty;       /* ���ռ�ձ� */
    volatile uint16_t   v_bus;          /* ���ߵ�ѹ mv */
    volatile uint16_t   v_t;         /* �������¶� �Ŵ�100�� 125 = 1.25���϶� */
    volatile uint16_t   brake_flag;     /* ɲ����־ */
    volatile uint16_t   brake_duty;     /* ɲ��ʱ��ռ�ձ� */
//    volatile uint32_t   hall_keep_t;    /* ��������ʱ�� */
//    volatile uint32_t   hall_pul_num;   /* ���������������� */
//    volatile uint32_t   lock_time;      /* �����תʱ�� */
//    volatile uint32_t   lock_time;      /* �����תʱ�� */
//    volatile uint32_t   no_single;
//    volatile uint32_t   count_j;
}_bldc_obj;


extern TIM_HandleTypeDef   g_MA_timx_handle;
extern TIM_HandleTypeDef   g_MB_timx_handle;

extern _bldc_obj g_bldc_motorA;
extern _bldc_obj g_bldc_motorB;

extern volatile uint32_t time_num;
int int_limit(int num,int min ,int max);
void bldc_init(uint16_t arr, uint16_t psc);




#endif
