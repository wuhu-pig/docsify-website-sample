//############################################################
// FILE:  ADC_int.h
// Created on: 2017��1��9��
// Author: XQ   
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/1/24
//�汾��V17.3-1
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#ifndef ADC_int_H
#define ADC_int_H

#include "stm32f10x.h"
 
typedef struct {
	     int32_t   BUS_Curr ;     // DC Bus  Current
	     int32_t   PhaseU_Curr;   // Phase A Current
	     int32_t   PhaseV_Curr;   // Phase A Current
	     int32_t   BUS_Voltage ;  //DC Bus  Voltage	     
	     int32_t   RP_speed_Voltage ;     //  RP1_Voltage
	     int32_t   OffsetBUS_Curr ;     // DC Bus  Current
	     int32_t   OffsetPhaseU_Curr;   // Phase A Current
	     int32_t   OffsetPhaseV_Curr;   // Phase A Current
	     int32_t   Coeff_filterK1;   // Phase A Current
		   int32_t   Coeff_filterK2;   // Phase A Current 
       }ADCSamp;

#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,268,756}

void  Offset_CurrentReading(void);
void  DMA_Configuration(void);
void  ADC1_Configuration(void);
void  ADC_Sample(void );
#endif  // end of ADC_int_H definition

//===========================================================================
// End of file.
//===========================================================================
