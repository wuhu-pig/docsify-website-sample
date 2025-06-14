//############################################################
// FILE:  ADC_int.c
// Created on: 2017��1��5��
// Author: XQ
// summary: ADCSampPare
// ADC���� ��ʹ�ö�ʱ��1�жϺ�������ADC�жϲ��� 
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
#include "ADC_int.h"
#include "GPIO_int.h"
#define ADC1_DR_Address    0x4001244C 

extern   ADCSamp     ADCSampPare;
uint16_t  ADC_ConvertedValue[5]={0};
 

void ADC1_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;	
    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* ADC1 DeInit */ 
    ADC_DeInit(ADC1);
 
    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);
		
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //����ת������
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 5;     //����ת�����г���Ϊ2
    ADC_Init(ADC1, &ADC_InitStructure);
    
		RCC_ADCCLKConfig(RCC_PCLK2_Div4); // 72/2   
  
    //����ת������1��ͨ��0    ����ʱ��>1.6us,(7cycles)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��1   
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
    //����ת������1��ͨ��2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��3   
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��8  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_7Cycles5);
  
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
    ADC_DMACmd(ADC1, ENABLE);
    
    // ������ADC�Զ�У׼����������ִ��һ�Σ���֤����
    // Enable ADC1 reset calibaration register 
    ADC_ResetCalibration(ADC1);
    // Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC1));

    // Start ADC1 calibaration
    ADC_StartCalibration(ADC1);
    // Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(ADC1));
    // ADC�Զ�У׼����---------------
   //������һ��ADת��
   ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
   //��Ϊ�Ѿ����ú���DMA��������AD�Զ�����ת��������Զ�������RegularConvData_Tab��   
    
}


void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
 	
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;	 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //BufferSize=2����ΪADCת��������2��ͨ��
    //������ã�ʹ����1�������RegularConvData_Tab[0]������2�������RegularConvData_Tab[1]
    DMA_InitStructure.DMA_BufferSize =5;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ����
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    //������ɺ�����DMAͨ��
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

 
 
//У׼����,������������ƫ��ֵΪ1.65V
void Offset_CurrentReading(void)
{
	static uint8_t i;  
 
  /* ADC Channel used for current reading are read  in order to get zero currents ADC values*/
  //16�β�����ƽ��ֵ��������������ʼУ׼ 
for(i=16; i!=0; i--)   
  {
   
    ADCSampPare.OffsetBUS_Curr += ADC_ConvertedValue[0];
 	  ADCSampPare.OffsetPhaseV_Curr += ADC_ConvertedValue[1];
    ADCSampPare.OffsetPhaseU_Curr += ADC_ConvertedValue[2];		 
   
    Delay(10000);
		Delay(10000);
  }
  ADCSampPare.OffsetBUS_Curr = ADCSampPare.OffsetBUS_Curr>>4;
	ADCSampPare.OffsetPhaseV_Curr= ADCSampPare.OffsetPhaseV_Curr>>4; 
	ADCSampPare.OffsetPhaseU_Curr=ADCSampPare.OffsetPhaseU_Curr>>4;
}
 
 

void   ADC_Sample(void )
{
  ADCSampPare.BUS_Curr  =       (ADC_ConvertedValue[0]-ADCSampPare.OffsetBUS_Curr)<<1;	
 	ADCSampPare.PhaseV_Curr  =    (ADC_ConvertedValue[1]-ADCSampPare.OffsetPhaseV_Curr)<<1;	    
  ADCSampPare.PhaseU_Curr  =    (ADC_ConvertedValue[2]-ADCSampPare.OffsetPhaseU_Curr)<<1;	
  ADCSampPare.RP_speed_Voltage = ADC_ConvertedValue[3];	
  ADCSampPare.BUS_Voltage   = ADC_ConvertedValue[4];	
 

}


//===========================================================================
// No more.
//===========================================================================
