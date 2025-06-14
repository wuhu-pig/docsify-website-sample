/**
 ****************************************************************************************************
 * @file        gtim.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-12-31
 * @brief       ͨ�ö�ʱ�� ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211231
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __GTIM_H
#define __GTIM_H

#include "./SYSTEM/sys/sys.h"
typedef struct {	
	      uint8_t   PWMZD_count;
        uint8_t   IntClock_1ms;	 
        uint8_t   Tim1ms_flag;	
				uint8_t   Tim10ms_count;	
        uint8_t   Tim10ms_flag;
        uint8_t   Tim100ms_count;
        uint8_t   Tim100ms_flag;
        uint8_t   Tim500ms_count;
        uint8_t   Tim500ms_flag; 
        uint8_t   Tim1s_count;
        uint8_t   Tim1s_flag ;
        uint8_t   Tim10s_count;
        uint8_t   Tim10s_flag ;
        uint8_t   Tim1min_count;
        uint8_t   Tim1min_flag ;
	   }TaskTime;

#define  TaskTime_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0}
/******************************************************************************************/
/* ͨ�ö�ʱ�� ���� */

/* TIMX �ж϶��� 
 * Ĭ�������TIM2~TIM5, TIM9~TIM14.
 * ע��: ͨ���޸���4���궨��,����֧��TIM1~TIM14����һ����ʱ��.
 */
 
#define GTIM_TIMX_INT                       TIM3
#define GTIM_TIMX_INT_IRQn                  TIM3_IRQn
#define GTIM_TIMX_INT_IRQHandler            TIM3_IRQHandler
#define GTIM_TIMX_INT_CLK_ENABLE()          do{ RCC->APB1ENR |= 1 << 1; }while(0)  /* TIM3 ʱ��ʹ�� */

/******************************************************************************************/
extern TaskTime TaskTimePare;
extern void gtim_timx_int_init(uint16_t arr, uint16_t psc);    /* ͨ�ö�ʱ�� ��ʱ�жϳ�ʼ������ */
extern void RunSystimer(void);
extern void CLEAR_flag(void);
#endif

















