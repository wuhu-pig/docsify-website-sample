#ifndef __SMO_H
#define __SMO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

// ��ģ�۲��������ṹ
typedef struct {
    float Rs;           // ���ӵ��� (��)
    float Ls;           // ���ӵ�� (H)
    float Kslf;         // ��ģ����
    float Freq;         // PWMƵ�� (Hz)
    uint16_t POLES;     // ���������
    float LPF_cutoff;   // ��ͨ�˲�����ֹƵ�� (Hz)
    float sat_boundary; // ���ͺ����߽�ֵ
} SMO_Params;

// ��ģ�۲���״̬�ṹ
typedef struct {
    float Ialpha_est;   // �����������ֵ
    float Ibeta_est;    // �����������ֵ
    float EMF_alpha;    // ���ᷴ�綯�ƹ���ֵ
    float EMF_beta;     // ���ᷴ�綯�ƹ���ֵ
    float theta;        // ת��λ�ã���Ƕȣ����ȣ�
    float speed;        // ת�٣�RPM��
    float EMF_alpha_fil; // �˲���Ħ��ᷴ�綯��
    float EMF_beta_fil;  // �˲���Ħ��ᷴ�綯��
    float theta_prev;    // ǰһ��ת��λ��
} SMO_Handle;

// ��ʼ����ģ�۲���
void SMO_Init(SMO_Handle *h);

// ��ģ�۲�������
void SMO_Update(SMO_Handle *h, SMO_Params *params, 
                float Ia, float Ib, float Ualpha, float Ubeta);

// ��ȡת��λ�ú��ٶ�
void SMO_GetPosition(SMO_Handle *h, SMO_Params *params);

// ���綯�Ƶ�ͨ�˲�
void SMO_EMF_Filter(SMO_Handle *h, SMO_Params *params);

// ���ͺ��������ٶ���
float sat(float x, float boundary);

#ifdef __cplusplus
}
#endif

#endif /* __SMO_H */