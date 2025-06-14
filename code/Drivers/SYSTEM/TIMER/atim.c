#include "atim.h"
#include "led.h"
#include "adc.h"
#include "foc.h"
/**
 * @brief       �߼���ʱ��TIMX ������� ��ʼ��������ʹ��PWMģʽ1��
 * @note
 *              ���ø߼���ʱ��TIMX �������, һ·OCy һ·OCyN, ���ҿ�����������ʱ��
 *
 *              �߼���ʱ����ʱ������APB2,��PPRE2��2��Ƶ��ʱ��
 *              �߼���ʱ����ʱ��ΪAPB2ʱ�ӵ�2��, ��APB2Ϊ84M, ���Զ�ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    /*A_Phase*/
    ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_A();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_A();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
		/*B_Phase*/
	  ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_B();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_B();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
		/*C_Phase*/
	   ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_C();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_C();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
	
    ATIM_TIMX_CPLM_BKIN_GPIO_CLK_ENABLE();  /* ͨ��Xɲ�������ӦIO��ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CLK_ENABLE();            /* ʹ�ܶ�ʱ��ʱ�� */

    sys_gpio_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_A, ATIM_TIMX_CPLM_CHY_GPIO_PIN_A,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHX ����ģʽ���� */
    sys_gpio_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_A, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_A,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHXN ����ģʽ���� */
	
    sys_gpio_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_B, ATIM_TIMX_CPLM_CHY_GPIO_PIN_B,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHX ����ģʽ���� */
    sys_gpio_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_B, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_B,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHXN ����ģʽ���� */ 
								 
		sys_gpio_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_C, ATIM_TIMX_CPLM_CHY_GPIO_PIN_C,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHX ����ģʽ���� */
    sys_gpio_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_C, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_C,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_CHXN ����ģʽ���� */
								 
    sys_gpio_set(ATIM_TIMX_CPLM_BKIN_GPIO_PORT, ATIM_TIMX_CPLM_BKIN_GPIO_PIN,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* TIMX_BKIN ����ģʽ���� */

    sys_gpio_af_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_A,  ATIM_TIMX_CPLM_CHY_GPIO_PIN_A,  ATIM_TIMX_CPLM_CHY_GPIO_AF_A);   /* IO�ڸ��ù���ѡ�� �������ö�!! */
    sys_gpio_af_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_A, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_A, ATIM_TIMX_CPLM_CHYN_GPIO_AF_A);  /* IO�ڸ��ù���ѡ�� �������ö�!! */
		
		sys_gpio_af_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_B,  ATIM_TIMX_CPLM_CHY_GPIO_PIN_B,  ATIM_TIMX_CPLM_CHY_GPIO_AF_B);   /* IO�ڸ��ù���ѡ�� �������ö�!! */
    sys_gpio_af_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_B, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_B, ATIM_TIMX_CPLM_CHYN_GPIO_AF_B);  /* IO�ڸ��ù���ѡ�� �������ö�!! */
		
		sys_gpio_af_set(ATIM_TIMX_CPLM_CHY_GPIO_PORT_C,  ATIM_TIMX_CPLM_CHY_GPIO_PIN_C,  ATIM_TIMX_CPLM_CHY_GPIO_AF_C);   /* IO�ڸ��ù���ѡ�� �������ö�!! */
    sys_gpio_af_set(ATIM_TIMX_CPLM_CHYN_GPIO_PORT_C, ATIM_TIMX_CPLM_CHYN_GPIO_PIN_C, ATIM_TIMX_CPLM_CHYN_GPIO_AF_C);  /* IO�ڸ��ù���ѡ�� �������ö�!! */
		
    sys_gpio_af_set(ATIM_TIMX_CPLM_BKIN_GPIO_PORT, ATIM_TIMX_CPLM_BKIN_GPIO_PIN, ATIM_TIMX_CPLM_BKIN_GPIO_AF);  /* IO�ڸ��ù���ѡ�� �������ö�!! */

    ATIM_TIMX_CPLM->ARR = arr;          /* �趨�������Զ���װֵ */
    ATIM_TIMX_CPLM->PSC = psc;          /* ����Ԥ��Ƶ�� */
		
		
		for(int chy =1;chy<4;chy++ )
		{
				if (chy <= 2)
				{
					ATIM_TIMX_CPLM->CCMR1 |= 6 << (4 + 8 * (chy - 1));  /* CH1/2 PWMģʽ1 */
					ATIM_TIMX_CPLM->CCMR1 |= 1 << (3 + 8 * (chy - 1));  /* CH1/2 Ԥװ��ʹ�� */
				}
				else if (chy <= 4)
				{
					ATIM_TIMX_CPLM->CCMR2 |= 6 << (4 + 8 * (chy - 3));  /* CH3/4 PWMģʽ1 */
					ATIM_TIMX_CPLM->CCMR2 |= 1 << (3 + 8 * (chy - 3));  /* CH3/4 Ԥװ��ʹ�� */
				}
					ATIM_TIMX_CPLM->CCER |= 1 << (4 * (chy - 1));       /* OCy ���ʹ�� */
					ATIM_TIMX_CPLM->CCER |= 1 << (1 + 4 * (chy - 1));   /* OCy �͵�ƽ��Ч */
					ATIM_TIMX_CPLM->CCER |= 1 << (2 + 4 * (chy - 1));   /* OCyN ���ʹ�� */
					ATIM_TIMX_CPLM->CCER |= 1 << (3 + 4 * (chy - 1));   /* OCyN �͵�ƽ��Ч */
		
		}

    // �ؼ��޸Ĳ���
    ATIM_TIMX_CPLM->CR1 &= ~(0x03 << 5);        // ���CMSλ
    ATIM_TIMX_CPLM->CR1 |=  (0x01 << 5);        // CMS=01 (���Ķ���ģʽ1)
    ATIM_TIMX_CPLM->CR1 |=  (0x01 << 7);        // ARPEʹ��
    ATIM_TIMX_CPLM->CR1 |=  (0x02 << 8);        // CKD=10 (ʱ�ӷ�Ƶ)

    // ����TRGO�����ĵ㴥��(������Ϊ0ʱ)
    ATIM_TIMX_CPLM->CR2 &= ~(0x07 << 4);        // ���MMSλ
    ATIM_TIMX_CPLM->CR2 |=  (0x02 << 4);        // MMS=010 (�����¼���ΪTRGO)

    // ɲ������
    ATIM_TIMX_CPLM->BDTR |= 0 << 16;            // BKF[3:0]=0 (���˲�)
    ATIM_TIMX_CPLM->BDTR |= 1 << 14;            // AOE=1 (�Զ��ָ����)
    ATIM_TIMX_CPLM->BDTR |= 0 << 13;            // BKP=0 (�͵�ƽ��Ч)
    ATIM_TIMX_CPLM->BDTR |= 1 << 12;            // BKE=1 (ɲ��ʹ��)

    // �ж�����
    ATIM_TIMX_CPLM->DIER |= 1 << 0;             // ʹ�ܸ����ж�(UIE)
    ATIM_TIMX_CPLM->SR  &= ~(1 << 0);           // ������±�־
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1);    // �����ж����ȼ�
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);         // ʹ���ж�ͨ��

    ATIM_TIMX_CPLM->CR1 |= 1 << 0;              // ʹ�ܶ�ʱ��
}

/**
 * @brief       ��ʱ��TIMX ��������Ƚ�ֵ & ����ʱ��
 * @param       ccr: ����Ƚ�ֵ
 * @param       dtg: ����ʱ��
 *   @arg       dtg[7:5]=0xxʱ, ����ʱ�� = dtg[7:0] * tDTS
 *   @arg       dtg[7:5]=10xʱ, ����ʱ�� = (64 + dtg[6:0]) * 2  * tDTS
 *   @arg       dtg[7:5]=110ʱ, ����ʱ�� = (32 + dtg[5:0]) * 8  * tDTS
 *   @arg       dtg[7:5]=111ʱ, ����ʱ�� = (32 + dtg[5:0]) * 16 * tDTS
 *   @note      tDTS = 1 / (Ft /  CKD[1:0]) = 1 / 42M = 23.8ns
 * @retval      ��
 */
void atim_timx_cplm_pwm_set(uint16_t ccr, uint8_t channel)
{
    /* ���� DTG[7:0] */
    ATIM_TIMX_CPLM->BDTR &= ~(0XFF << 0);
    /* ���� DTG[7:0] */
    ATIM_TIMX_CPLM->BDTR |= 100 << 0;
    /* ʹ������� */
    ATIM_TIMX_CPLM->BDTR |= 1 << 15;
    /* ���ñȽϼĴ��� */
    switch(channel) {
        case 1:
            ATIM_TIMX_CPLM_CHY_CCRY_A = ccr;
            break;
        case 2:
            ATIM_TIMX_CPLM_CHY_CCRY_B = ccr;
            break;
        case 3:
            ATIM_TIMX_CPLM_CHY_CCRY_C = ccr;
            break;
        default:
            break;
    }
}

// ��stm32f4xx_it.c�����
void TIM8_UP_TIM13_IRQHandler(void)
{
    if (ATIM_TIMX_CPLM->SR & TIM_SR_UIF) {
        // ����жϱ�־
        ATIM_TIMX_CPLM->SR &= ~TIM_SR_UIF;
				LED0_TOGGLE();
				foc_main();
       // adc_dma_enable(ADC_DMA_BUF_SIZE);
        // ����������Ĵ������
        // ���磺HAL_GPIO_TogglePin(GPIOx, GPIO_PIN_x); // ������
    }
}
