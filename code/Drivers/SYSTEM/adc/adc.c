#include "adc.h"
#include "led.h"
#include "dma.h"
#include "delay.h"
#include "foc.h"
#include "usart.h"
#include "math.h"

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE]; 
uint8_t g_adc_dma_sta = 0;  /* DMA传输状态标志, 0,未完成; 1, 已完成 */
uint16_t adc_buffer[ADC_CHANNEL_NUM]; 

// INA4181A3IPWR 配置参数
#define INA4181_GAIN          50.0f    // 增益: 50V/V (A3版本)
#define SHUNT_RESISTANCE      0.05f    // 采样电阻: 50mΩ
#define ADC_REFERENCE_VOLTAGE 3.3f     // ADC参考电压
#define ADC_RESOLUTION        4096.0f  // 12位ADC分辨率

// 电流零点偏移校准值
float current_offset[ADC_CHANNEL_NUM];

// 过流保护配置
#define OVERCURRENT_THRESHOLD 20.0f    // 过流阈值: 20A
#define OVERCURRENT_HYSTERESIS 2.0f    // 过流滞回: 2A
static uint8_t overcurrent_flag = 0;   // 过流标志

void adc_init(void)
{
    // 1. 时钟使能
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;     // DMA2时钟
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // ADC1时钟

    // 2. 配置PA0、PA1、PA2为模拟输入
    GPIOA->MODER |= (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2));  // 设置为模拟模式

    // 3. 配置ADC时钟
    ADC->CCR |= (1 << 16); // PCLK2 / 4 = 84MHz / 4 = 21MHz

    // 4. 配置ADC工作模式
    ADC1->CR1 = ADC_CR1_SCAN;                            // 启用扫描模式
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS;               // DMA模式 + DMA自动请求
    ADC1->CR2 &= ~ADC_CR2_EOCS;                          // 只在序列完成后触发EOC
    ADC1->CR2 |= (1 << 0);                               // 开启ADC
    ADC1->CR2 &= ~(1 << 1);                              // 关闭连续转换（使用外部触发）

    // 5. 配置外部触发：TIM8_TRGO（EXTSEL = 0b1110）
    ADC1->CR2 &= ~(0xF << 24);                           // 清除EXTSEL
    ADC1->CR2 |= (0xE << 24);                            // EXTSEL = 0b1110
    ADC1->CR2 |= (1 << 28);                              // EXTEN = 0b01 => 上升沿触发

    // 6. 配置通道序列：PA0(IN0)、PA1(IN1)、PA2(IN2)
    ADC1->SQR1 &= ~(0xF << 20);                          // 清除L位
    ADC1->SQR1 |= (2 << 20);                             // L = 2 => 3个通道 (0,1,2)
    ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10);       // 顺序 IN0 → IN1 → IN2 (PA0 → PA1 → PA2)
}

void dma_init(void)
{
    DMA2_Stream0->CR = 0;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);              // 等待DMA关闭
    DMA2->LIFCR |= 0x3D << 0;                           // 清除所有通道0中断标志

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;            // ADC数据寄存器
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;          // 内存地址
    DMA2_Stream0->NDTR = ADC_CHANNEL_NUM;               // 数据数量 = 3
    DMA2_Stream0->CR |= (0 << 6);                       // 外设到内存
    DMA2_Stream0->CR |= (1 << 8);                       // 外设地址不变
    DMA2_Stream0->CR |= (1 << 10);                      // 内存地址递增
    DMA2_Stream0->CR |= (1 << 11);                      // 外设数据宽度：16位
    DMA2_Stream0->CR |= (1 << 13);                      // 内存数据宽度：16位
    DMA2_Stream0->CR |= (0 << 16);                      // 单次模式
    DMA2_Stream0->CR |= (0 << 21);                      // 通道选择：通道0（ADC1）
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;                  // 开启传输完成中断

    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    DMA2_Stream0->CR |= DMA_SxCR_EN;                    // 启动DMA
}

// 获取原始电流值（未校准）
float get_phase_current_raw(uint8_t channel)
{
    if (channel >= ADC_CHANNEL_NUM) return 0.0f;
    
    // ADC原始值转换为电压
    float voltage = (float)(adc_buffer[channel]) * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION;
    
    // 计算采样电阻上的电压降
    float shunt_voltage = voltage / INA4181_GAIN;
    
    // 根据欧姆定律计算电流: I = V/R
    float current = shunt_voltage / SHUNT_RESISTANCE;
    
    return current;
}

// 获取电流值（转换为实际电流值）
float get_phase_current(uint8_t channel)
{
    if (channel >= ADC_CHANNEL_NUM) return 0.0f;
    
    // 获取原始电流值
    float current = get_phase_current_raw(channel);
 
    return current;
}

// 获取三相电流
void get_three_phase_currents(float* ia, float* ib, float* ic)
{
    *ia = get_phase_current(0)-current_offset[0];  // PA0 - A相电流
    *ib = get_phase_current(1)-current_offset[1];  // PA1 - B相电流  
    *ic = get_phase_current(2)-current_offset[2];  // PA2 - C相电流
}

// FOC电流环集成示例
void foc_current_control_example(float* ia, float* ib, float* ic)
{ 
    // 获取三相电流
    get_three_phase_currents(ia, ib, ic);
    
    // 过流检测
    if (check_overcurrent(*ia, *ib, *ic)) {
        // 过流保护：停止电机
        printf("过流保护触发！Ia=%.2fA, Ib=%.2fA, Ic=%.2fA\r\n", *ia, *ib, *ic);
        
        // 这里可以调用电机停止函数
        // motor_emergency_stop();
        
        // 设置电流为0
        *ia = 0.0f;
        *ib = 0.0f;
        *ic = 0.0f;
        
        return;  // 跳过电流环控制
    }
    
    // 电流滤波（可选）
    static float ia_filtered = 0, ib_filtered = 0, ic_filtered = 0;
    float filter_coeff = 0.1f;  // 低通滤波系数
    
    ia_filtered = ia_filtered * (1.0f - filter_coeff) + (*ia) * filter_coeff;
    ib_filtered = ib_filtered * (1.0f - filter_coeff) + (*ib) * filter_coeff;
    ic_filtered = ic_filtered * (1.0f - filter_coeff) + (*ic) * filter_coeff;
    
    // 将滤波后的电流值传回
    *ia = ia_filtered;
    *ib = ib_filtered;
    *ic = ic_filtered;
    
    // 将电流值传递给FOC控制算法
    // 这里可以调用你的FOC函数，例如：
    // foc_current_control(ia_filtered, ib_filtered, ic_filtered);
    
    // 或者直接更新电机结构体中的电流值
    // my_motor.current.ia = ia_filtered;
    // my_motor.current.ib = ib_filtered;
    // my_motor.current.ic = ic_filtered;
}

void DMA2_Stream0_IRQHandler(void)
{
    if(DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;  // 清除中断标志
        // 处理ADC结果
//        LED0_TOGGLE();
      if(MOTOR_CLOSED_LOOP==motor_state)
			{
//				LED0_TOGGLE();
				// 获取三相电流
				get_three_phase_currents((float *)&my_motor.phasecurrent.ia,(float *)&my_motor.phasecurrent.ib,(float *)&my_motor.phasecurrent.ic);
				foc_main_spwm();
				
			}
    }
}

// 电流校准函数
void calibrate_current_offset(void)
{
    const int samples = 16;  // 采集256个样本求平均
    float sum[ADC_CHANNEL_NUM] = {0, 0, 0};
    
    printf("开始电流零点校准\r\n");
    printf("请确保电机停止运行\r\n");
    
    // 采集多次样本求平均
    for(int i = 0; i < samples; i++) {
        delay_ms(10);  // 等待ADC稳定
        
        for(int ch = 0; ch < ADC_CHANNEL_NUM; ch++) {
            float current = get_phase_current_raw(ch);  // 获取原始电流值
            sum[ch] += current;
        }
    }
    
    // 计算平均值作为零点偏移
    for(int ch = 0; ch < ADC_CHANNEL_NUM; ch++) {
        current_offset[ch] = sum[ch] /samples;
        printf("通道%d零点偏移: %f\r\n", ch, current_offset[ch]);
    }
		
    printf("电流零点校准完成\r\n");
		
		motor_state=MOTOR_STATUS_OK;
}

// 过流检测函数
uint8_t check_overcurrent(float ia, float ib, float ic)
{
    float max_current = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
    
    if (max_current > OVERCURRENT_THRESHOLD) {
        overcurrent_flag = 1;
        return 1;  // 过流
    } else if (max_current < (OVERCURRENT_THRESHOLD - OVERCURRENT_HYSTERESIS)) {
        overcurrent_flag = 0;
        return 0;  // 正常
    }
    
    return overcurrent_flag;  // 保持之前状态
}

// 获取过流状态
uint8_t get_overcurrent_status(void)
{
    return overcurrent_flag;
}

// 清除过流标志
void clear_overcurrent_flag(void)
{
    overcurrent_flag = 0;
}







