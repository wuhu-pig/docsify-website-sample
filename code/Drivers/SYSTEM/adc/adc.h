#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"

#define ADC_DMA_BUF_SIZE        50 * 6          /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */

typedef struct {
	     int32_t   BUS_Curr ;     // DC Bus  Current
	     int32_t   PhaseU_Curr;   // Phase A Current
	     int32_t   PhaseV_Curr;   // Phase B Current
			 int32_t   PhaseW_Curr;   // Phase B Current
	     int32_t   BUS_Voltage ;  //DC Bus  Voltage	     
	     int32_t   RP_speed_Voltage ;     //  RP1_Voltage
	     int32_t   OffsetBUS_Curr ;     // DC Bus  Current
	     int32_t   OffsetPhaseU_Curr;   // Phase A Current
	     int32_t   OffsetPhaseV_Curr;   // Phase B Current
			 int32_t   OffsetPhaseW_Curr;   // Phase B Current
		   int32_t   EA_Curr;   // Phase A Current
	     int32_t   EB_Curr;   // Phase B Current
			 int32_t   EC_Curr;   // Phase B Current
		   int32_t   OffsetEA_Curr;   // Phase A Current
	     int32_t   OffsetEB_Curr;   // Phase B Current
			 int32_t   OffsetEC_Curr;   // Phase B Current
	     int32_t   Coeff_filterK1;   // Phase A Current
		   int32_t   Coeff_filterK2;   // Phase B Current 
			 int32_t   Coeff_filterK3;   // Phase B Current 
	}ADCSamp;

#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,268,756}

/******************************************************************************************/
/* ADC������ ���� */

#define ADC_ADCX_CHY_GPIO_PORT              GPIOA
#define ADC_ADCX_CHY_GPIO_PIN               SYS_GPIO_PIN5 
#define ADC_ADCX_CHY_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 0; }while(0)   /* PA��ʱ��ʹ�� */

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CHY                        5                                       /* ͨ��Y,  0 <= Y <= 19 */ 
#define ADC_ADCX_CHY_CLK_ENABLE()           do{ RCC->APB2ENR |= 1 << 8; }while(0)   /* ADC1 ʱ��ʹ�� */
#define ADC_CHANNEL_NUM  5  // ����ɼ�3·ADCͨ��


/* ADC��ͨ��/��ͨ�� DMA�ɼ� DMA��������� ���� 
 * ע��: �������ǵ�ͨ������ʹ������Ķ���.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_Channel              0                                       /* ͨ��0 */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()              ( DMA2->HISR & (1 << 5) )   /* �ж� DMA2_Stream4 ������ɱ�־, ����һ���ٺ�����ʽ,
                                                                         * ���ܵ�����ʹ��, ֻ������if��������� 
                                                                         */
#define ADC_ADCX_DMASx_CLR_TC()             do{ DMA2->HIFCR |= 1 << 5; }while(0)    /* ��� DMA2_Stream4 ������ɱ�־ */

/******************************************************************************************/
extern  uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];       /* ADC DMA BUF */
extern uint8_t g_adc_dma_sta;     
extern uint16_t adc_buffer[ADC_CHANNEL_NUM];  // DMA����Ŀ���ַ
extern void adc_init(void);
extern void dma_init(void);
#endif 















