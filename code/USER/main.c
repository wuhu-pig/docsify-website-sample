#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h" 
#include "atim.h"
#include "gtim.h"
#include "adc.h"
#include "math.h"
#include "i2c.h"
#include "spi.h"
#include "foc.h"
#include "pid.h"

void Motorstatuswsitch(void);	

int main(void)
{
		/*变量定义*/
		static uint8_t value;
		uint16_t angle11;
		/**********/
	
		/*初始化函数*/
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(84, 115200);                 /* 串口初始化为115200 */
    led_init(); 														//LED初始化                         	
		SPI2_GPIO_MasterInit();
    SPI2_MasterInit();
		AS5600_Init();
		init_moving_means();
		adc_init();
    dma_init();
    atim_timx_cplm_pwm_init(PWM_PERIOD - 1, 168 - 1); /* 168/4=42Mhz的计数频率 1Khz的周期. */  //1000对应1ms 50对应50us
		foc_init();
		motor_state=MOTOR_PAREE;
		gtim_timx_int_init(PWM_PERIOD-1, 84 - 1); /* 84 000 000 / 84 00 = 10 000 10Khz的计数频率，计数5K次为500ms */	//100对应100us
		/**************/
		while (1)
    {
				if(TaskTimePare.Tim100us_flag==1)//100us任务
				{          
					TaskTimePare.Tim100us_flag=0;
					/*100us任务添加*/
					//LED0_TOGGLE();
					value++;
					if(value>=2)
					{
						value=0;
						anglevaluesum=AS5600_ReadRawAngle();
						int diff=anglevaluesum-angle11;
						if(diff>4096)
						{
							anglecycle++;
							diff-=4096;
							angle11=anglevaluesum;
						}else if (diff<-4096){
							anglecycle--;
							diff+=4096;
							angle11=anglevaluesum;
						}
						
					}
					/*************/
					if(++TaskTimePare.Tim100us_count >=10)
					{
						TaskTimePare.Tim100us_count=0;
						TaskTimePare.Tim1ms_flag=1;
					}
				}
				if(TaskTimePare.Tim1ms_flag==1)//1ms任务
				{          
					TaskTimePare.Tim1ms_flag=0;
					/*1ms任务添加*/
					/*************/
					if(++TaskTimePare.Tim1ms_count >=10)
					{
						TaskTimePare.Tim1ms_count=0;
						TaskTimePare.Tim10ms_flag=1;
					}
				}
				if(TaskTimePare.Tim10ms_flag==1)//10ms任务
				{	
					TaskTimePare.Tim10ms_flag=0;
					/*10ms任务添加*/
					Motorstatuswsitch();//电机状态切换
					
					/*************/
					if(++TaskTimePare.Tim10ms_count >=10)
					{
						TaskTimePare.Tim10ms_count=0;
						TaskTimePare.Tim100ms_flag=1;
					}
				}
				if(TaskTimePare.Tim100ms_flag==1)//100ms任务
				{
					TaskTimePare.Tim100ms_flag=0;

					/*100ms任务添加*/
					
					/***************/
				}
    }
}

void Motorstatuswsitch(void)
{
		switch(motor_state){
				case MOTOR_PAREE:
						calibrate_current_offset();
				break;
				case MOTOR_CURRENT_CAL:
						break;
				case MOTOR_STATUS_OK:
						break;
				case MOTOR_STOPPED:
						break;
				case MOTOR_ALIGNMENT:
						break;
				case MOTOR_OPEN_LOOP:
						break;
				case MOTOR_CLOSED_LOOP:
						break;
				default:
					break;
			}
}
