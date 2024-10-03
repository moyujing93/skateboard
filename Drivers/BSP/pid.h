/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com/forum.php
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  1         /* 0：位置式 ，1：增量式 */

#if INCR_LOCT_SELECT

/* 转速PID参数相关宏 */
#define  KP_S      0.002f               /* P参数 0.05*/
#define  KI_S      0.002f               /* I参数 0.02*/
#define  KD_S      0.00f                /* D参数*/

/* 电流PID参数相关宏 */
#define  KP_C      0.0015f               /* P参数  0.005*/
#define  KI_C      0.001f               /* I参数  0.002*/
#define  KD_C      0.00f                /* D参数*/


#else

/* 位置式转速PID参数相关宏 */
#define  KP      8.50f               /* P参数*/
#define  KI      5.00f               /* I参数*/
#define  KD      0.10f               /* D参数*/
#endif

/* PID参数结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern uint8_t  pid_sta;
extern PID_TypeDef  g_MA_speed_pid;     /*速度环PID参数结构体*/
extern PID_TypeDef  g_MA_current_pid;   /* 电流环PID参数结构体 */

extern PID_TypeDef  g_MB_speed_pid;     /*速度环PID参数结构体*/
extern PID_TypeDef  g_MB_current_pid;   /* 电流环PID参数结构体 */
/******************************************************************************************/

void pid_init(uint16_t hz );                 /* pid初始化 */

#endif
