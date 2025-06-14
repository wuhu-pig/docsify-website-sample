//############################################################
// FILE: GPIO_int.c
// Created on: 2017��1��18��
// Author: XQ
// summary: GPIO_int
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

#ifndef _GPIO_int_H
#define _GPIO_int_H
#include "stm32f10x.h"
 
//���κ꣬��������������һ��ʹ��
#define LED_485RE_ON   	GPIO_SetBits(GPIOB,GPIO_Pin_2) 	 
#define LED_485RE_OFF   GPIO_ResetBits(GPIOB,GPIO_Pin_2) 
 
void Delay(u32 nCount); 
void Init_Gpio_ADC(void);
void GPIO_LED485RE_int(void);	
void Init_Gpio_TIM1_PWM(void);
void InitThreeHallGpio(void ); 
void Init_Encoder_Gpio (void);
void Init_PWMDAC_Gpio (void);
void InitCAN_Gpio(void );
void InitUSART3_Gpio(void );
void LED1_Toggle(void); 
#endif   //  GPIO_int.h
//===========================================================================
// No more.
//===========================================================================
