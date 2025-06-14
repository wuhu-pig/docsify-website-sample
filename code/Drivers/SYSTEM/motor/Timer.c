//############################################################
// FILE: Timer.c
// Created on: 2017��1��11��
// Author: XQ    
// summary: Timer    
// ��ʱ��1������ƣ������������Ķ�ʱ��IO    
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
#include "Timer.h"
#include "GPIO_int.h"
#include "Tim1_PWM.h"

extern  TaskTime       TaskTimePare;    
 
void SysTickConfig(void)
{
  /* Setup SysTick Timer for 1ms interrupts  */
  if (SysTick_Config(SystemCoreClock /100))   //  10ms
  {
    /* Capture error */ 	
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}
 
void RunSystimer(void)
{
 if(TaskTimePare.IntClock_10ms==1)
    {          
     TaskTimePare.IntClock_10ms=0;
	   TaskTimePare.Tim10ms_flag = 1;
	 if(++TaskTimePare.Tim100ms_count >=10)
	   {
	    TaskTimePare.Tim100ms_count=0;
	    TaskTimePare.Tim100ms_flag=1;
	   }
    }
       if(TaskTimePare.Tim100ms_flag==1)
	   {		   		 
	    if(++TaskTimePare.Tim500ms_count >=5)
	    {	      
	       TaskTimePare.Tim500ms_count=0;
	       TaskTimePare.Tim500ms_flag=1;
      }
	   }
        if(TaskTimePare.Tim500ms_flag==1)
	   {            
	    if(++TaskTimePare.Tim1s_count >=2)
	    {	               
	       TaskTimePare.Tim1s_count=0;
	       TaskTimePare.Tim1s_flag=1;
      }
	   }
      if(TaskTimePare.Tim1s_flag == 1)
	   {      
         LED1_Toggle( );   
	   if(++TaskTimePare.Tim10s_count >=10)
	    {
		TaskTimePare.Tim10s_count = 0;
		TaskTimePare.Tim10s_flag = 1;
	    }
	   }
  
        if(TaskTimePare.Tim10s_flag == 1)
	   {                   
	   if(++TaskTimePare.Tim1min_count >=6)
	    {
	  	TaskTimePare.Tim1min_count = 0;
		  TaskTimePare.Tim1min_flag = 1;
	    }
	   }
}

void CLEAR_flag(void)
{
	TaskTimePare.Tim10ms_flag=0;
	TaskTimePare.Tim100ms_flag=0;
	TaskTimePare.Tim500ms_flag=0;
	TaskTimePare.Tim1s_flag=0;
	TaskTimePare.Tim1min_flag=0;
}


//===========================================================================
// No more.
//===========================================================================
