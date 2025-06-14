#include "gtim.h"
#include "led.h"
#include "motor.h"
TaskTime   TaskTimePare=TaskTime_DEFAULTS;

void CLEAR_flag(void)
{
	TaskTimePare.Tim1ms_flag=0;
	TaskTimePare.Tim10ms_flag=0;
	TaskTimePare.Tim100ms_flag=0;
	TaskTimePare.Tim500ms_flag=0;
	TaskTimePare.Tim1s_flag=0;
	TaskTimePare.Tim1min_flag=0;
}
void RunSystimer(void)
{
 if(TaskTimePare.IntClock_1ms==1)
    {          
			TaskTimePare.IntClock_1ms=0;
			TaskTimePare.Tim10ms_flag = 1;
			if(++TaskTimePare.Tim10ms_count >=10)
	   {
	    TaskTimePare.Tim10ms_count=0;
	    TaskTimePare.Tim10ms_flag=1;
	   }
    }
		   if(TaskTimePare.Tim10ms_flag==1)
	   {		   		 
	    if(++TaskTimePare.Tim100ms_count >=10)
	    {	      
	       TaskTimePare.Tim100ms_count=0;
	       TaskTimePare.Tim100ms_flag=1;
      }
	   }
       if(TaskTimePare.Tim100ms_flag==1)
	   {		   		 
	    if(++TaskTimePare.Tim500ms_count >=5)
	    {	      
	       TaskTimePare.Tim500ms_count=0;
	       TaskTimePare.Tim500ms_flag=1;
      }
	   }
        if(TaskTimePare.Tim500ms_flag==1)
	   {            
	    if(++TaskTimePare.Tim1s_count >=2)
	    {	               
	       TaskTimePare.Tim1s_count=0;
	       TaskTimePare.Tim1s_flag=1;
      }
	   }
      if(TaskTimePare.Tim1s_flag == 1)
	   {      
			if(++TaskTimePare.Tim10s_count >=10)
	    {
				TaskTimePare.Tim10s_count = 0;
				TaskTimePare.Tim10s_flag = 1;
	    }
	   }
  
      if(TaskTimePare.Tim10s_flag == 1)
	   {                   
				if(++TaskTimePare.Tim1min_count >=6)
	    {
				TaskTimePare.Tim1min_count = 0;
				TaskTimePare.Tim1min_flag = 1;
	    }
	   }
}
/**
 * @brief       ͨ�ö�ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void GTIM_TIMX_INT_IRQHandler(void)
{ 
    if (GTIM_TIMX_INT->SR & 0X0001)   /* ����ж� */
    {
				GTIM_TIMX_INT->SR &= ~(1 << 0); /* ����жϱ�־λ */
        
				TaskTimePare.IntClock_1ms=1;
//				LED1_TOGGLE();
				//motor_foc_algorithm_main();
//				ADC_Sample();
				
				TaskTimePare.Tim1ms_flag=1;
    }

} 

/**
 * @brief       ͨ�ö�ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ͨ�ö�ʱ����ʱ������APB1,��PPRE1��2��Ƶ��ʱ��
 *              ͨ�ö�ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void gtim_timx_int_init(uint16_t arr, uint16_t psc)
{
    GTIM_TIMX_INT_CLK_ENABLE();
    GTIM_TIMX_INT->ARR = arr;           /* �趨�������Զ���װֵ */
    GTIM_TIMX_INT->PSC = psc;           /* ����Ԥ��Ƶ��  */
    GTIM_TIMX_INT->DIER |= 1 << 0;      /* ��������ж� */
    GTIM_TIMX_INT->CR1 |= 1 << 0;       /* ʹ�ܶ�ʱ��TIMX */
    sys_nvic_init(1, 3, GTIM_TIMX_INT_IRQn, 2); /* ��ռ1�������ȼ�3����2 */
}










