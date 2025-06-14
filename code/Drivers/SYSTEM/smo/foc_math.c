#include "foc_math.h"

// ����atan2����ʵ��
float fast_atan2(float y, float x) {
    float abs_y = fabsf(y) + 1e-20f; // ���������
    float angle;
    
    if (x >= 0) {
        float r = (x - abs_y) / (x + abs_y);
        angle = PI_4 - PI_4 * r;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        angle = PI_3_4 - PI_4 * r;
    }
    
    return (y < 0) ? -angle : angle;
}

// Clarke�任����������£�
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta) {
    *Ialpha = Ia;
    *Ibeta = (Ia + 2 * Ib) * ONE_BY_SQRT3;  // 1/��3 �� 0.577
}

// Park�任�����¡�dq��
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq) {
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);
    *Id = Ialpha * cos_t + Ibeta * sin_t;
    *Iq = -Ialpha * sin_t + Ibeta * cos_t;
}

// ��Park�任��dq�����£�
void Inv_Park_Transform(float Vd, float Vq, float theta, float *Valpha, float *Vbeta) {
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);
    *Valpha = Vd * cos_t - Vq * sin_t;
    *Vbeta = Vd * sin_t + Vq * cos_t;
}

// SVPWM����
void SVPWM_Generate(float Valpha, float Vbeta, float Vdc, 
                    float *Ta, float *Tb, float *Tc) {
    // ���������ѹ
    float Va = Valpha;
    float Vb = -0.5f * Valpha + SQRT3_BY_2 * Vbeta;
    float Vc = -0.5f * Valpha - SQRT3_BY_2 * Vbeta;
    
    // ����ƫ����ʹ����Ϊ��
    float Vmin = fminf(fminf(Va, Vb), Vc);
    float Vmax = fmaxf(fmaxf(Va, Vb), Vc);
    float Voffset = (Vmax + Vmin) * 0.5f;
    
    // Ӧ��ƫ�Ʋ���һ����0-1��Χ
    *Ta = (Va - Voffset) / Vdc + 0.5f;
    *Tb = (Vb - Voffset) / Vdc + 0.5f;
    *Tc = (Vc - Voffset) / Vdc + 0.5f;
}