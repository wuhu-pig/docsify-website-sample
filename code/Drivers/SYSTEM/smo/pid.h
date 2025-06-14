#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

// PID�������ṹ
typedef struct {
    float Kp;           // ��������
    float Ki;           // ��������
    float Kd;           // ΢������
    float integral;     // ������
    float prev_error;   // ǰһ�����
    float output;       // ���ֵ
    float output_max;   // �������
    float output_min;   // �������
    float integral_max; // ����������
} PID_Controller;

// ��ʼ��PID������
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, 
              float output_min, float output_max, float integral_max);

// ����PID������
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */