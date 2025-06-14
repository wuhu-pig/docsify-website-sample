#ifndef __ATIM_H
#define __ATIM_H

#include "sys.h"

/******************************************************************************************/
/*
 * ע��: ͨ���޸���Щ�궨��,����֧��TIM1/TIM8��ʱ��, ����һ��IO���������PWM(ǰ���Ǳ����л����������)
 */

/* A_Phase���ͨ������ */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT_A            GPIOC
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN_A            	SYS_GPIO_PIN6
#define ATIM_TIMX_CPLM_CHY_GPIO_AF_A              2                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_A()    do{ RCC->AHB1ENR |= 1 << 2; }while(0)       /* PE��ʱ��ʹ�� */

/* A_Phase�������ͨ������ */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT_A           GPIOA
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN_A            SYS_GPIO_PIN7
#define ATIM_TIMX_CPLM_CHYN_GPIO_AF_A             4                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_A()   do{ RCC->AHB1ENR |= 1 << 0; }while(0)       /* PE��ʱ��ʹ�� */

/* B_Phase���ͨ������ */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT_B            GPIOC
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN_B            	SYS_GPIO_PIN7
#define ATIM_TIMX_CPLM_CHY_GPIO_AF_B              2                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_B()    do{ RCC->AHB1ENR |= 1 << 2; }while(0)       /* PE��ʱ��ʹ�� */

/* BPhase�������ͨ������ */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT_B           GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN_B            SYS_GPIO_PIN14
#define ATIM_TIMX_CPLM_CHYN_GPIO_AF_B             3                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_B()   do{ RCC->AHB1ENR |= 1 << 4; }while(0)       /* PE��ʱ��ʹ�� */

/* C_Phase���ͨ������ */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT_C            GPIOC
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN_C            	SYS_GPIO_PIN8
#define ATIM_TIMX_CPLM_CHY_GPIO_AF_C              2                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE_C()    do{ RCC->AHB1ENR |= 1 << 2; }while(0)       /* PE��ʱ��ʹ�� */

/* C_Phase�������ͨ������ */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT_C           GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN_C            SYS_GPIO_PIN15
#define ATIM_TIMX_CPLM_CHYN_GPIO_AF_C             3                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE_C()   do{ RCC->AHB1ENR |= 1 << 4; }while(0)       /* PE��ʱ��ʹ�� */

/* ɲ���������� */
#define ATIM_TIMX_CPLM_BKIN_GPIO_PORT           	GPIOA
#define ATIM_TIMX_CPLM_BKIN_GPIO_PIN            	SYS_GPIO_PIN6
#define ATIM_TIMX_CPLM_BKIN_GPIO_AF             	4                                           /* AF����ѡ�� */
#define ATIM_TIMX_CPLM_BKIN_GPIO_CLK_ENABLE()   	do{ RCC->AHB1ENR |= 1 << 0; }while(0)       /* PE��ʱ��ʹ�� */

/* �������ʹ�õĶ�ʱ�� */
#define ATIM_TIMX_CPLM                          		TIM8
#define ATIM_TIMX_CPLM_CHY_A                      	1
#define ATIM_TIMX_CPLM_CHY_CCRY_A                 	ATIM_TIMX_CPLM->CCR1
#define ATIM_TIMX_CPLM_CHY_B                      	2
#define ATIM_TIMX_CPLM_CHY_CCRY_B                 	ATIM_TIMX_CPLM->CCR2
#define ATIM_TIMX_CPLM_CHY_C                      	3
#define ATIM_TIMX_CPLM_CHY_CCRY_C                 	ATIM_TIMX_CPLM->CCR3
#define ATIM_TIMX_CPLM_CLK_ENABLE()             	do{ RCC->APB2ENR |= 1 << 1; }while(0)       /* TIM2 ʱ��ʹ�� */
/******************************************************************************************/

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);   /* �߼���ʱ�� ������� ��ʼ������ */
void atim_timx_cplm_pwm_set(uint16_t ccr, uint8_t channel);
#endif













