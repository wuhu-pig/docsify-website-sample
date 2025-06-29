#ifndef __PID_H
#define __PID_H
#include <stdint.h>
#include <string.h>

#define WINDOW_SIZE 8  // 必须是2的幂（16, 32, 64等）
#define SHIFT_BITS 3    // log2(WINDOW_SIZE)
typedef struct
{
    float MovingMean;   // 移动均值后的值
    float newvalue;     // 当前数值
    float oldvalue;     // 之前均值后的值
} float_MovingMean;

typedef struct
{
    uint32_t MovingMean;   // 移动均值后的值
    uint32_t newvalue;     // 当前数值
    uint32_t oldvalue;     // 之前均值后的值
    
    // 新增环形缓冲区状态
    uint32_t buffer[WINDOW_SIZE];  // 环形缓冲区
    uint32_t sum;                  // 当前窗口内数据的总和
    uint32_t index;                // 当前写入位置索引
} uint_MovingMean;

typedef struct{
	float err; //误差值
	float err_prev; //上两次误差值
	float err_last; //上一次误差值
	float integral;
	float Kp; //比例系数
	float Ki; //积分系数
	float Kd; //微分系数
	float Time;
	float output;
}PID;
#define PIDDefParam {0.0f,0.0f,0.0f,0.0f,5.0f,3.0f,2.0f,1.0f,.0f,}


extern float_MovingMean fmechanicalangle;
extern uint_MovingMean umechanicalangle;
extern uint32_t anglevaluesum;
extern uint16_t anglecycle;
extern PID anglepid;


void init_moving_means(void);
void MovingMeanalgo(float_MovingMean * Mean);
void uintMovingMeanalgo(uint_MovingMean * Mean);


float anglepidfun(PID *pid,float current,float target);

#endif
