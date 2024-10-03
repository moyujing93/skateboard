/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID�㷨����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com/forum.php
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID��ز��� */

#define  INCR_LOCT_SELECT  1         /* 0��λ��ʽ ��1������ʽ */

#if INCR_LOCT_SELECT

/* ת��PID������غ� */
#define  KP_S      0.002f               /* P���� 0.05*/
#define  KI_S      0.002f               /* I���� 0.02*/
#define  KD_S      0.00f                /* D����*/

/* ����PID������غ� */
#define  KP_C      0.0015f               /* P����  0.005*/
#define  KI_C      0.001f               /* I����  0.002*/
#define  KD_C      0.00f                /* D����*/


#else

/* λ��ʽת��PID������غ� */
#define  KP      8.50f               /* P����*/
#define  KI      5.00f               /* I����*/
#define  KD      0.10f               /* D����*/
#endif

/* PID�����ṹ�� */
typedef struct
{
    __IO float  SetPoint;            /* �趨Ŀ�� */
    __IO float  ActualValue;         /* �������ֵ */
    __IO float  SumError;            /* ����ۼ� */
    __IO float  Proportion;          /* �������� P */
    __IO float  Integral;            /* ���ֳ��� I */
    __IO float  Derivative;          /* ΢�ֳ��� D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern uint8_t  pid_sta;
extern PID_TypeDef  g_MA_speed_pid;     /*�ٶȻ�PID�����ṹ��*/
extern PID_TypeDef  g_MA_current_pid;   /* ������PID�����ṹ�� */

extern PID_TypeDef  g_MB_speed_pid;     /*�ٶȻ�PID�����ṹ��*/
extern PID_TypeDef  g_MB_current_pid;   /* ������PID�����ṹ�� */
/******************************************************************************************/

void pid_init(uint16_t hz );                 /* pid��ʼ�� */

#endif
