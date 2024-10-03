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
#ifndef  __BLDC_SIX_STEP_H
#define  __BLDC_SIX_STEP_H

#include "./SYSTEM/sys/sys.h"




void MA_uhvl(void);
void MA_uhwl(void);
void MA_vhwl(void);
void MA_vhul(void);
void MA_whul(void);
void MA_whvl(void);
void MA_stop(void);
void MA_break(void);

void MB_uhvl(void);
void MB_uhwl(void);
void MB_vhwl(void);
void MB_vhul(void);
void MB_whul(void);
void MB_whvl(void);
void MB_stop(void);
void MB_break(void);






#endif
