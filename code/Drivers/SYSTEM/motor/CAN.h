//############################################################
// FILE: CAN.h
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

#ifndef __CAN_H
#define __CAN_H 
 
#include "stm32f10x.h"
#include "stm32f10x_can.h"

typedef struct {
	
        uint32_t   ext_Rece_ID;	 
        uint32_t   ext_Send_ID;
        uint8_t    Can_Send_data[8];
        uint8_t    Can_Rece_data[8];
        uint8_t    set_tem;    
        uint8_t     Cycle_mode;
        uint8_t    fanmode_set;
        uint8_t    infanspeed_set;
        uint8_t    chuschuw;
        uint8_t    CAN_rx_flag;
	   }CANSR;

#define  CANSR_DEFAULTS  {0x18FA0170,0x18FA0171,0,0,0,0,0,0,0,0}

void CAN_Receivechuli(void);
void CAN_Config(void); 
void CAN_Sendlen(void);
void USB_LP_CAN1_RX0_IRQHandler(void);

#endif /* __CAN_H */
