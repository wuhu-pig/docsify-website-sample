//############################################################
// FILE: Tim1_PWM.h
// Created on: 2017��1��18��
// Author: XQ
// summary: Tim1_PWM
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
   
#ifndef _Tim1_PWM_H
#define _Tim1_PWM_H 

#include "IQ_math.h"
#include "./SYSTEM/sys/sys.h"

#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
#define PWM_PRSC ((u8)0)                          
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) //72000000/(2*12500*(1))=2880  2880=PWM 100%
/****	Power devices switching frequency  ****/

#define PWM_FREQ ((u16)12500) // in Hz  (N.b.: pattern type is center aligned),�����������ʱ12.5kHZ���������ģʽ��
#define PWM_HalfPerMax   ((u16)1440)
#define REP_RATE (0)  
 // (N.b): Internal current loop is performed every (REP_RATE + 1)/(2*PWM_FREQ) seconds.               
 // REP_RATE has to be an odd number in case of three-shunt
 // current reading; this limitation doesn't apply to ICS
#define DEADTIME  (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL)
#define DEADTIME_NS	((u16) 1800)  //in nsec; range is [0...3500]  

#define DCBUS_VOLTAGE			(int32_t)_IQ(24)		//   Q10��ʽ,ĸ�ߵ�ѹ24V  24576 

#define STATOR_VOLTAGE		 (int32_t)_IQ(13.867) 		// Udc/��3* Q15  13.867   14190  ��ʽ   

#define MAX_STATOR_VOLTAGE	(int32_t)_IQ(13.5)  	 // Udc/��3*0.97 13.467   13765 Q15��ʽ  



void  Svpwm_Outpwm(void);
void Tim1_PWM_Init(void);
void Stop_Motor(void);
void Start_Motor(void);
#endif /* __TIMER_H */

//===========================================================================
// No more.
//===========================================================================
