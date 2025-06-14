#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "smo.h"
#include "pid.h"

// �������״̬��
typedef enum {
    MOTOR_STOPPED,      // ���ֹͣ
    MOTOR_ALIGNMENT,    // Ԥ��λ
    MOTOR_OPEN_LOOP,    // ��������
    MOTOR_CLOSED_LOOP   // �ջ�����
} Motor_State;

// ������ƽṹ
typedef struct {
    Motor_State state;          // ��ǰ״̬
    uint32_t start_time;        // ����ʱ��
    float open_loop_angle;      // �����Ƕ�
    float open_loop_freq;       // ����Ƶ��
    float open_loop_voltage;    // ������ѹ
    SMO_Handle smo;             // ��ģ�۲���
    SMO_Params smo_params;      // SMO����
    PID_Controller pid_speed;   // �ٶȻ�PID
    PID_Controller pid_id;      // d�������PID
    PID_Controller pid_iq;      // q�������PID
} Motor_Control;

// ��ʼ���������
void MotorControl_Init(Motor_Control *mc);

// ���������ѭ��
void MotorControl_Update(Motor_Control *mc, 
                         float Ia, float Ib, float Ic, 
                         float Vdc, uint32_t timestamp);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */