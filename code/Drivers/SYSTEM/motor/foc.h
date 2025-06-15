#ifndef __FOC_H
#define __FOC_H

#include "sys.h"

#define PI          	3.14159265358979323846f
#define TWO_PI     	 	(2.0f * PI)
#define SQRT3       	1.73205080757f
#define ONE_BY_SQRT3 	0.57735026919f
#define SQRT3_BY_2  	0.86602540378f

#define RAMPSTEP 			0.01
#define MAXSPEED 			5					//��λ rad/s  2*2pi/60=2*2*180/60=12��/s
#define Ts 						0.001			//1ms
#define Vref 					12				//��Դ��ѹ12v
#define Vrefby2 			6					//��Դ��ѹ12v

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// ���������ṹ��
typedef struct {
    float ia;
    float ib;
    float ic;
} PhaseCurrent_t;


// ���������ṹ��
typedef struct {
    float va;
    float vb;
    float vc;
} PhaseVoltage_t;

// ���������ṹ��
typedef struct {
    uint16_t ccra;
    uint16_t ccrb;
    uint16_t ccrc;
} PhaseCcr_t;

// ��ѹ�����ṹ��
typedef struct {
    float vd;
    float vq;
} VoltageDQ_t;

// ���������ṹ��
typedef struct {
    float id;
    float iq;
} CurrentDQ_t;

// ��ѹ�����ṹ��
typedef struct {
    float valpha;
    float vbeta;
} Voltageabeta_t;

// ���������ṹ��
typedef struct {
    float ialpha;
    float ibeta;
} Currentabeta_t;

// �����������ṹ��
typedef struct {
    float angle_elec;  // ��Ƕ�
    float speed_rpm;   // ת�٣���ѡ��
} MotorFeedback_t;

// PI �������ṹ��
typedef struct {
    float kp;
    float ki;
    float integral;
    float output_limit;
} PI_Controller_t;

// ��ģ�۲��������ṹ
typedef struct {
	//�����������г�
		uint16_t Poles;     	// ���������
		uint16_t KV;					//KVֵ
		float Maxcurrent;			//���ɳ��ܵ���
		float Maxvoltage;			//����ѹ
		float Phaseresitance;
		float Wireresistance;
		float WireLs;
		float	PhaseLs;
	
    float Rs;           	// ���ӵ��� (��)
    float Ls;          	 	// ���ӵ�� (H)
    float Kslf;        	 	// ��ģ����
    float Freq;         	// PWMƵ�� (Hz)
    float LPF_cutoff;   	// ��ͨ�˲�����ֹƵ�� (Hz)
    float sat_boundary; 	// ���ͺ����߽�ֵ
} Motor_Params;

// �������״̬��
typedef enum {
    MOTOR_STOPPED,      // ���ֹͣ
    MOTOR_ALIGNMENT,    // Ԥ��λ
    MOTOR_OPEN_LOOP,    // ��������
    MOTOR_CLOSED_LOOP   // �ջ�����
} Motor_State;

// �������״̬��
typedef struct {
	float targetspeed;
	float targetposition;
	float targetcurrent;
	float speed;
	float speed_el;
	float angle_el;
	float position;
	VoltageDQ_t voltage0;
	Voltageabeta_t voltage1;
	PhaseVoltage_t voltage2;
	PhaseCcr_t  CCR;
	float current;
	float maxcurrent;
 	float maxspeed;
} Motor_Control;

typedef struct{
	PhaseCurrent_t	phasecurrent;
	VoltageDQ_t 		voltage;
	CurrentDQ_t 		dqcurrent;
	Motor_Params  	params;
	PI_Controller_t pid;
	Motor_State  state;
	Motor_Control control;
}Motor_t; 

extern Motor_t my_motor;
extern void foc_init(void);
extern void foc_main(void);
extern void inverse_clarke_transform_with_3rd_harmonic(float alpha, float beta, float *a, float *b, float *c);

#endif
