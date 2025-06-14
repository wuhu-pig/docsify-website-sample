//############################################################
// FILE:   Usart_RS232.h
// Created on: 2017��1��21��
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
 
#ifndef Usart_RS232_H
#define Usart_RS232_H
#include "IQ_math.h"
#include "stm32f10x.h"

typedef struct {		     
        uint8_t   Uart_txdr[8];
        uint8_t   Uart_rxdr[8];
	      uint16_t  fact_BUS_Curr;
	      uint16_t  fact_BUS_Voil;
		    uint16_t  iq_test;
	      uint16_t  id_test;
	      uint16_t  uq_test;
	      uint16_t  ud_test;
	      int16_t   IqRef_test;
	      uint16_t  Speed_target;
	      uint16_t  Speed_fact;
        uint8_t   Rece_flag;
	      uint8_t   Rxdr_byte;
        uint8_t   chaoshi_jishu;
        uint8_t   chaoshi_pand;
        uint8_t   chaoshi_pandold;
        uint8_t   chaoshi_shu;
	   }Test;

#define  Test_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
 
void Uart3_RS232TX_sen(void);
void Usart3_RS232_init(void);
void Uart2_Sendlen(uint8_t *data,u8 len);
void USART3_IRQHandler(void);  
void  ReceiveData_chuli(void);
#endif  // end of SCI_RS232.h definition

//===========================================================================
// End of file.
//===========================================================================
