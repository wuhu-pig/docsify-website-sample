//############################################################
// FILE: CAN.c
// Created on: 2017��1��18��
// Author: XQ
// summary: ThreeHall
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

#include "CAN.h"
#include "timer.h"

extern  TaskTime   TaskTimePare;  
extern  CANSR      CANSRPare;
 
CanTxMsg   TxMessage = {0}; 
CanRxMsg   RxMessage= {0}; 
 
void CAN_Config(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;  
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0?D??
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //?��??��??��??0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			   //������??��???a0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // ��Ҫ
   
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
 
  CAN_InitStructure.CAN_TTCM=DISABLE;			 
  CAN_InitStructure.CAN_ABOM=DISABLE;			   
  CAN_InitStructure.CAN_AWUM=DISABLE;			 	
  CAN_InitStructure.CAN_NART=DISABLE;			   
  CAN_InitStructure.CAN_RFLM=DISABLE;			    
  CAN_InitStructure.CAN_TXFP=DISABLE;			     
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	   
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		    
  CAN_InitStructure.CAN_BS1=CAN_BS1_15tq;		   
  CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;		 
  CAN_InitStructure.CAN_Prescaler =6;    //  72/ (1+15+8)/6/2 =250

  CAN_Init(CAN1, &CAN_InitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber=0;						 
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;		 
  																  																//1: 1y???�¡���x��?2??32????��??��1�����?��������?��?��D����?�꨺??��
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	 																	//0��o1y???��???��?a2??16??��? 1��o1y???��???��?a�̣�??32???�� 
  CAN_FilterInitStructure.CAN_FilterIdHigh=((CANSRPare.ext_Rece_ID <<3)>>16) &0xffff ;  // 32  0x6420    0x05A0<<5					
  CAN_FilterInitStructure.CAN_FilterIdLow=(CANSRPare.ext_Rece_ID <<3)| CAN_ID_EXT ;  // 0x2461  
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF ;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0XFFF8;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;				 
  																	//??��?��?��?��???1?��a��?FIFO?D?�� 0��o1y???�¡�?1?��a��?FIFO0��? 1��o1y???�¡�?1?��a��?FIFO1?��
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;				 
  																	//?��??CAN_FMR??��??�¦�?FINIT??����??1o����?2??��DT???����|��?1y???��??��??��																	//x(CAN_FxR[0:1])?�� 0��o1y???�¡�???��?��? 1��o1y???�¡�??��???��
  CAN_FilterInit(&CAN_FilterInitStructure);
 
  TxMessage.StdId = 0 ;    //      18FA0171
  TxMessage.ExtId = CANSRPare.ext_Send_ID;
  TxMessage.RTR = CAN_RTR_DATA;  // 0  CAN_RTR_DATA
  TxMessage.IDE =CAN_ID_EXT ;   //  0  CAN_ID_EXT
  TxMessage.DLC = 8;   
 
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); 
}


void CAN_Receivechuli(void)
{
  uint8_t i=0; 
  if (CANSRPare.CAN_rx_flag==1 )
  {  
    CANSRPare.CAN_rx_flag=0;       
    for(i=0;i<8;i++)	
		CANSRPare.Can_Rece_data[i] = CANSRPare.Can_Rece_data[i];    //  16----  27    
  }  
}

 
void CAN_Sendlen(void)
{     
       uint8_t i=0;  
       if(TaskTimePare.Tim1s_flag==1)    
       {       
        CANSRPare.Can_Send_data[0]=0xAA ;// run_sta        
        CANSRPare.Can_Send_data[1]=0x1F;         
        CANSRPare.Can_Send_data[2]=0x12 ;         
        CANSRPare.Can_Send_data[3]=0x3F;
        CANSRPare.Can_Send_data[4]=0x65 ;
        CANSRPare.Can_Send_data[5]=0x40 ;
        CANSRPare.Can_Send_data[6]=0x18;      						
	      CANSRPare.Can_Send_data[7]=0xFF;			
     
        for(i=0;i<8;i++)						
        TxMessage.Data[i] =  CANSRPare.Can_Send_data[i];	
     
       CAN_Transmit(CAN1, &TxMessage);         //transmit_mailbox   ��Ҫ 
       }  
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
 uint8_t n=0;         
  
  if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0) 
  {   
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    CANSRPare.CAN_rx_flag=1;     
    for(n=0;n<8;n++  )      
    CANSRPare.Can_Rece_data[n] = RxMessage.Data[n];   
  }  
}

 
