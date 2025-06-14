#include "adc.h"
#include "led.h"
#include "dma.h"
#include "delay.h"

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE]; 
uint8_t g_adc_dma_sta = 0;  /* DMA����״̬��־, 0,δ���; 1, ����� */
uint16_t adc_buffer[ADC_CHANNEL_NUM]; 

void adc_init(void)
{
    // 1. ʱ��ʹ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOAʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;     // DMA2ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // ADC1ʱ��

    // 2. ����PA0~PA2Ϊģ������
    GPIOA->MODER |= (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2));

    // 3. ����ADCʱ��
    ADC->CCR |= (1 << 16); // PCLK2 / 4 = 84MHz / 4 = 21MHz

    // 4. ����ADC����ģʽ
    ADC1->CR1 = ADC_CR1_SCAN;                            // ����ɨ��ģʽ
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS;               // DMAģʽ + DMA�Զ�����
    ADC1->CR2 &= ~ADC_CR2_EOCS;                          // ֻ��������ɺ󴥷�EOC
    ADC1->CR2 |= (1 << 0);                               // ����ADC
    ADC1->CR2 &= ~(1 << 1);                              // �ر�����ת����ʹ���ⲿ������

    // 5. �����ⲿ������TIM8_TRGO��EXTSEL = 0b1110��
    ADC1->CR2 &= ~(0xF << 24);                           // ���EXTSEL
    ADC1->CR2 |= (0xE << 24);                            // EXTSEL = 0b1110
    ADC1->CR2 |= (1 << 28);                              // EXTEN = 0b01 => �����ش���

    // 6. ����ͨ������
    ADC1->SQR1 &= ~(0xF << 20);                          // ���Lλ
    ADC1->SQR1 |= (4 << 20);                             // L = 2 => 3��ͨ��
    ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 10) | (4 << 10) | (5 << 10);        // ˳�� IN0 �� IN1 �� IN2 �� IN3 �� IN4 �� IN5
}

void dma_init(void)
{
    DMA2_Stream0->CR = 0;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);              // �ȴ�DMA�ر�
    DMA2->LIFCR |= 0x3D << 0;                           // �������ͨ��0�жϱ�־

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;            // ADC���ݼĴ���
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;          // �ڴ��ַ
    DMA2_Stream0->NDTR = ADC_CHANNEL_NUM;               // �������� = 3
    DMA2_Stream0->CR |= (0 << 6);                       // ���赽�ڴ�
    DMA2_Stream0->CR |= (1 << 8);                       // �����ַ����
    DMA2_Stream0->CR |= (1 << 10);                      // �ڴ��ַ����
    DMA2_Stream0->CR |= (1 << 11);                      // �������ݿ�ȣ�16λ
    DMA2_Stream0->CR |= (1 << 13);                      // �ڴ����ݿ�ȣ�16λ
    DMA2_Stream0->CR |= (0 << 16);                      // ����ģʽ
    DMA2_Stream0->CR |= (0 << 21);                      // ͨ��ѡ��ͨ��0��ADC1��
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;                  // ������������ж�

    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    DMA2_Stream0->CR |= DMA_SxCR_EN;                    // ����DMA
}


void DMA2_Stream0_IRQHandler(void)
{
    if(DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;  // ����жϱ�־

        // ����ADC���
			LED0_TOGGLE();

        // �û������߼�...
    }
}







