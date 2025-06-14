//############################################################
// FILE: Timer.h
// Created on: 2017��1��18��
// Author: XQ
// summary: Timer_ 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/1/23
//�汾��V17.3-1
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################
   
#ifndef _TIMER_H
#define _TIMER_H 

#include "stm32f10x.h"

typedef struct {	
	      uint8_t   PWMZD_count;	
        uint8_t   IntClock_10ms;	 
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
   
void SysTickConfig(void);
void RunSystimer(void);
void CLEAR_flag(void);

#endif /* __TIMER_H */

//===========================================================================
// No more.
//===========================================================================
