//############################################################
// FILE: GPIO_int.c
// Created on: 2017��1��18��
// Author: XQ
// summary: GPIO_int
// LED  495RE CANͨѶIO������ͨѶIO ��ʱ��1�������IO��AD�ڳ�ʼ�� ������IO�������������Ķ�ʱ��IO
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
#include "GPIO_int.h"

void Delay(u32 nCount)	 //�򵥵���ʱ����
{
	u16 t=10000;
	for(; nCount != 0; nCount--)//������ʱ�����Ǻ�0�Ƚ�
	for(;t!=0;t--);
} 

void GPIO_LED485RE_int(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure; 	 
 /* Enable GPIOA-GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC,ENABLE);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
}

 void InitThreeHallGpio(void ) 
{
  GPIO_InitTypeDef GPIO_InitStructure; 	 
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void InitCAN_Gpio(void ) 
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  //  CAN TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  //  CAN RX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void InitUSART3_Gpio(void ) 
{
  GPIO_InitTypeDef GPIO_InitStructure; 	 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;	/* USART3 Tx (PB.10)*/							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/******************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;	/* USART3 Rx (PB.11)*/								
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
 
void Init_PWMDAC_Gpio (void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  /*Configure PB.06,07 as encoder input */	
  GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_Gpio_ADC(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure; 		
 //ĸ��ƽ������PA0, B�����PA1,  A�����PA2,  ��λ��PA3�ٶ��ź�����, ĸ�ߵ�ѹPB0, �˿ڳ�ʼ�� 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 	 
}
 
void Init_Gpio_TIM1_PWM(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure; 	
  
/*Timer1 alternate function full remapping*/ //Timer1���ù�������������ӳ�� 
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	
/* GPIOA,GPIOB, Configuration: Channel 1, 1N, 2, 2N, 3 and 3N Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;//IR2136�ĸߣ�1��2��3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//IR2136�ͣ�1N��2N��3N
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	/* Lock GPIOB Pin9 to Pin 13 *///Ҫ���Ͷ�
  //GPIO_PinLockConfig(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);	 
 // GPIO_StructInit(&GPIO_InitStructure); 
	
		/*PB12�˿���Ϊɲ������˿� */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 }
 
void LED1_Toggle(void)       
{     
  GPIOB->BSRR = GPIO_Pin_2 << (((GPIOB->ODR & GPIO_Pin_2)==0)?0:16); 
}  
 
//===========================================================================
// No more.
//===========================================================================
