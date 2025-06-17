/**
 ****************************************************************************************************
 * @file        exti.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-12-30
 * @brief       外部中断 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211230
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __EXTI_H
#define __EXTI_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 和 中断编号 & 中断服务函数 定义 */ 

#define KEY0_INT_GPIO_PORT              GPIOE
#define KEY0_INT_GPIO_PIN               SYS_GPIO_PIN4
#define KEY0_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE口时钟使能 */
#define KEY0_INT_IRQn                   EXTI4_IRQn
#define KEY0_INT_IRQHandler             EXTI4_IRQHandler

#define KEY1_INT_GPIO_PORT              GPIOE
#define KEY1_INT_GPIO_PIN               SYS_GPIO_PIN3
#define KEY1_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE口时钟使能 */
#define KEY1_INT_IRQn                   EXTI3_IRQn
#define KEY1_INT_IRQHandler             EXTI3_IRQHandler

#define KEY2_INT_GPIO_PORT              GPIOE
#define KEY2_INT_GPIO_PIN               SYS_GPIO_PIN2
#define KEY2_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE口时钟使能 */
#define KEY2_INT_IRQn                   EXTI2_IRQn
#define KEY2_INT_IRQHandler             EXTI2_IRQHandler

#define WKUP_INT_GPIO_PORT              GPIOA
#define WKUP_INT_GPIO_PIN               SYS_GPIO_PIN0
#define WKUP_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 0; }while(0)   /* PA口时钟使能 */
#define WKUP_INT_IRQn                   EXTI0_IRQn
#define WKUP_INT_IRQHandler             EXTI0_IRQHandler

/******************************************************************************************/


void extix_init(void);  /* 外部中断初始化 */

#endif

























