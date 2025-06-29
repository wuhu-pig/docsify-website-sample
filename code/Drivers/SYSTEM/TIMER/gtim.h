#ifndef __GTIM_H
#define __GTIM_H

#include "./SYSTEM/sys/sys.h"
typedef struct { 
				uint8_t   Tim100us_flag;
				uint8_t   Tim100us_count;		
        uint8_t   Tim1ms_flag;	
				uint8_t   Tim1ms_count;	
				uint8_t   Tim10ms_count;	
        uint8_t   Tim10ms_flag;
        uint8_t   Tim100ms_count;
        uint8_t   Tim100ms_flag;
	   }TaskTime;

#define  TaskTime_DEFAULTS  {0,0,0,0,0,0}
/******************************************************************************************/
/* 通用定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM2~TIM5, TIM9~TIM14.
 * 注意: 通过修改这4个宏定义,可以支持TIM1~TIM14任意一个定时器.
 */
 
#define GTIM_TIMX_INT                       TIM3
#define GTIM_TIMX_INT_IRQn                  TIM3_IRQn
#define GTIM_TIMX_INT_IRQHandler            TIM3_IRQHandler
#define GTIM_TIMX_INT_CLK_ENABLE()          do{ RCC->APB1ENR |= 1 << 1; }while(0)  /* TIM3 时钟使能 */

/******************************************************************************************/
extern TaskTime TaskTimePare;
extern void gtim_timx_int_init(uint16_t arr, uint16_t psc);    /* 通用定时器 定时中断初始化函数 */
extern void RunSystimer(void);
#endif

