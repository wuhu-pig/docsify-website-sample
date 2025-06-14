//############################################################
// FILE:  Task_function.h
// Created on: 2017��1��5��
// Author: XQ
// summary: Header file  and definition
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

#ifndef Task_function_H
#define Task_function_H

#include "IQ_math.h"
#include "stm32f10x.h" 

typedef struct {
	    uint8_t    Control_Mode;  // ����ģʽ  VF=1    �˱��뿪��=2      �˱���FOC=3      ������=4
	    uint8_t    TripFlagDMC;   //  ���� ������־
	    uint8_t    drive_car;
	    uint8_t    olddrive_car;
	    uint8_t    clear_PWMtripz;
	    uint8_t    Run_mode;
	    uint16_t    Qiehuan_count;
	    uint8_t    Start_order;
	   	 }logic;

#define  logic_DEFAULTS  {0,0,0,0,0,0,0,0}
 
void knob_control(void);   // ��ť��λ������ �ٶ��ź�

#endif  // end of Task_function_H definition

//===========================================================================
// End of file.
//===========================================================================
