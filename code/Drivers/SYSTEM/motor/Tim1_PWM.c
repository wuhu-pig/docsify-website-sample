//############################################################
// FILE: Tim1_PWM.c
// Created on: 2017��1��15��
// Author: XQ
// summary: Tim1_PWM
//  ��ʱ��1������ƣ������������Ķ�ʱ��IO    
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
#include "Tim1_PWM.h"
#include "Svpwm_dq.h"
//#include "Tim4_Encoder_PWMDAC.h"

//extern   EQEP   EQEPPare; 
extern   SVPWM    Svpwmdq;

int16_t  PWM_DUTY[3]={0,0,0};

void  Tim1_PWM_Init(void)
{ 
  TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
  TIM_OCInitTypeDef TIM1_OCInitStructure;
  TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
//TIM1ʱ����Ԫ����12500KHZ,һ��PWM������80us
  TIM1_TimeBaseStructure.TIM_Prescaler=0;
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1 ;//�������ģʽ 1 ����ģʽ
  TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;//Ҳ����ARRԤװ�ؼĴ��������ֵ=2250��
  TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//�����˲�������Ƶ��
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;//1��PWM���ڸ���һ��
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
	
	/*����ΪPWM����Ƚ�ģʽ*/
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //�����ȵ���ģʽ 1
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   
  	
  TIM1_OCInitStructure.TIM_Pulse =1000; //�Ƚ�ֵ
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //����ߵ�ƽ��Ч
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; 
  TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
 	
  TIM1_OCInitStructure.TIM_Pulse = 1000; //�Ƚ�ֵ
  TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
 
  TIM1_OCInitStructure.TIM_Pulse = 1000; //dummy value
  TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
 
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
  
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 

  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;//
  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;//�͵�ƽ������Ч
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;  

  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);
  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);  //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Break ,ENABLE); //ʹ���ж� 
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Update ,ENABLE); //���ж� 
 
  TIM_Cmd(TIM1, ENABLE);//������ʼ
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

/* Configure one bit for preemption priority */
	//ѡ�����ȼ�����
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//����TIM1��ɲ���ж�ʹ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ָ����ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//ָ����ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	//����TIM1�ĸ����ж�ʹ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//ָ����ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//ָ����Ӧ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
}

void  Svpwm_Outpwm(void)
{
   PWM_DUTY[0]= _IQmpy(PWM_HalfPerMax,Svpwmdq.Ta)+ PWM_HalfPerMax;
   PWM_DUTY[1]= _IQmpy(PWM_HalfPerMax,Svpwmdq.Tb)+ PWM_HalfPerMax;
	 PWM_DUTY[2]= _IQmpy(PWM_HalfPerMax,Svpwmdq.Tc)+ PWM_HalfPerMax;

	 TIM1->CCR1 = PWM_DUTY[0];//PWM_DUTY[0]  (EQEPPare.JZElecTheta>>5)
   TIM1->CCR2 = PWM_DUTY[1];//PWM_DUTY[1]
   TIM1->CCR3 = PWM_DUTY[2] ;//PWM_DUTY[2]
}


void Start_Motor(void)
{
  /*PWM�Ĵ���ռ�ձ�����*/
   TIM1->CCR1=0x00;
   TIM1->CCR2=0x00;
   TIM1->CCR3=0x00;
   //ʹ��PWM���ͨ��OC1/OC1N/OC2/OC2N/OC3/OC3N
   TIM1->CCER|=0x5555;	
}
 
void Stop_Motor(void)
{
  /*PWM�Ĵ���ռ�ձ�����*/
   TIM1->CCR1=PWM_PERIOD;
   TIM1->CCR2=PWM_PERIOD;
   TIM1->CCR3=PWM_PERIOD;
   //��ʹ��PWM���ͨ��OC1/OC1N/OC2/OC2N/OC3/OC3N
   TIM1->CCER&=0xAAAA;	
}

//===========================================================================
// No more.
//===========================================================================
