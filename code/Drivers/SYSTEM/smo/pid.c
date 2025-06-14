#include "pid.h"

// ��ʼ��PID������
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, 
              float output_min, float output_max, float integral_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_max = integral_max;
}

// ����PID������
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt) {
    // �������
    float error = setpoint - measurement;
    
    // ������
    float proportional = pid->Kp * error;
    
    // ������������ͣ�
    pid->integral += pid->Ki * error * dt;
    
    // ���ƻ�����
    if (pid->integral > pid->integral_max) 
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) 
        pid->integral = -pid->integral_max;
    
    // ΢����
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // �������
    pid->output = proportional + pid->integral + derivative;
    
    // �������
    if (pid->output > pid->output_max) 
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min) 
        pid->output = pid->output_min;
    
    return pid->output;
}