#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#define PI          3.14159265358979323846f
#define TWO_PI      (2.0f * PI)
#define SQRT3       1.73205080757f
#define ONE_BY_SQRT3 0.57735026919f
#define SQRT3_BY_2  0.86602540378f

// ����atan2�������ȱ�׼atan2��5����
float fast_atan2(float y, float x);

// Clarke�任����������£�
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);

// Park�任�����¡�dq��
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq);

// ��Park�任��dq�����£�
void Inv_Park_Transform(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);

// SVPWM����
void SVPWM_Generate(float Valpha, float Vbeta, float Vdc, 
                    float *Ta, float *Tb, float *Tc);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MATH_H */