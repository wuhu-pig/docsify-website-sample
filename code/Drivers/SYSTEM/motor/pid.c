#include "pid.h"
#include "AS5600.h"

// 选择精度更高的固定点实现
#define USE_FIXED_POINT

#ifdef USE_FIXED_POINT
    #define FIXED_SHIFT 8      // 8位小数精度
    #define ALPHA_02 51        // 0.2 * 256
    #define ALPHA_08 205       // 0.8 * 256
    #define ROUND (1 << (FIXED_SHIFT - 1))
#else
    #define ALPHA_SHIFT 4      // 基本移位实现
    #define ROUND (1 << (ALPHA_SHIFT - 1))
#endif

// 全局变量
float_MovingMean fmechanicalangle;
uint_MovingMean umechanicalangle;
uint32_t anglevaluesum;
uint16_t anglecycle;

PID anglepid=PIDDefParam;
// 初始化
void init_moving_means() {
    fmechanicalangle.MovingMean = 0.0f;
    fmechanicalangle.newvalue = 0.0f;
    fmechanicalangle.oldvalue = 0.0f;
    
    umechanicalangle.MovingMean = 0;
    umechanicalangle.newvalue = 0;
    umechanicalangle.oldvalue = 0;
}

// 浮点移动均值算法
void MovingMeanalgo(float_MovingMean * Mean) {
    Mean->MovingMean = Mean->newvalue * 0.2f + Mean->oldvalue * 0.8f;
    Mean->oldvalue = Mean->MovingMean;
}

// 整数移动均值算法
void uintMovingMeanalgo(uint_MovingMean * Mean) {
    #ifdef USE_FIXED_POINT
        // 固定点实现
        uint32_t new_part = (Mean->newvalue * ALPHA_02 + ROUND) >> FIXED_SHIFT;
        uint32_t old_part = (Mean->oldvalue * ALPHA_08 + ROUND) >> FIXED_SHIFT;
        Mean->MovingMean = new_part + old_part;
    #else
        // 移位实现
        uint32_t new_part = (Mean->newvalue + ROUND) >> ALPHA_SHIFT;
        uint32_t old_alpha = (Mean->oldvalue + ROUND) >> ALPHA_SHIFT;
        uint32_t old_part = Mean->oldvalue - old_alpha;
        Mean->MovingMean = new_part + old_part;
    #endif
    
    // 更新旧值
    Mean->oldvalue = Mean->MovingMean;
}

float anglepidfun(PID* pid,float current,float target)
{
	pid->err=target-current;//计算当前误差err[n]
	float derivative= pid->err-pid->err_last;//计算err[n]-err[n-1]
	float derivative1= pid->err-2*pid->err_last+pid->err_prev;//计算err[n]-2*err[n-1]+err[n-2]
	pid->output = pid->Kp*derivative+
									pid->Ki*pid->err*pid->Time+
										pid->Kd*derivative1;
	pid->err_prev=pid->err_last;
	pid->err_last=pid->err;
	return pid->output;
}


float PID_Update_standard(PID* pid, float current, float target) {
    // 计算当前误差
    float err = target - current;
    
    // 积分项累积（防饱和处理可选）
   pid-> integral =pid->err * pid->Time;
    
    // 微分项 = (当前误差 - 上次误差) / 采样时间
    float derivative = (err - pid->err_last) / pid->Time;
    
    // PID输出
    pid->output = pid->Kp * err 
                + pid->Ki * pid->integral 
                + pid->Kd * derivative;
    
    // 更新历史误差
    pid->err_last = err;
    
    return pid->output;
}


float PID_Update(PID* pid, float current, float target) {
    float err = target - current;
    
    // 计算各分量
    float P = pid->Kp * (err - pid->err_last);
    float I = pid->Ki * err * pid->Time;
    float D = pid->Kd * (err - 2*pid->err_last + pid->err_prev) / pid->Time;
    
    // 输出增量
    float delta_output = P + I + D;
    
    // 更新误差历史
    pid->err_prev = pid->err_last;
    pid->err_last = err;
    
    return delta_output;  // 注意：外部需累加此增量
}

