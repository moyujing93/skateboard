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
