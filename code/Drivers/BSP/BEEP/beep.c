/**
 ****************************************************************************************************
 * @file        beep.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-12-30
 * @brief       蜂鸣器 驱动代码
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

#include "./BSP/BEEP/beep.h"


/**
 * @brief       初始化BEEP相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void beep_init(void)
{
    BEEP_GPIO_CLK_ENABLE(); /* BEEP时钟使能 */

    sys_gpio_set(BEEP_GPIO_PORT, BEEP_GPIO_PIN,
                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);   /* BEEP引脚模式设置 */

    BEEP(0);    /* 关闭蜂鸣器 */
}






