#include "smo.h"
#include "foc_math.h" // ������ѧ���ߺ���

// ��ʼ����ģ�۲���
void SMO_Init(SMO_Handle *h) {
    h->Ialpha_est = 0.0f;
    h->Ibeta_est = 0.0f;
    h->EMF_alpha = 0.0f;
    h->EMF_beta = 0.0f;
    h->theta = 0.0f;
    h->speed = 0.0f;
    h->EMF_alpha_fil = 0.0f;
    h->EMF_beta_fil = 0.0f;
    h->theta_prev = 0.0f;
}

// ���ͺ��������ٶ���
float sat(float x, float boundary) {
    if (x > boundary) 
        return 1.0f;
    else if (x < -boundary) 
        return -1.0f;
    else 
        return x / boundary;
}

// ��ģ�۲�������
void SMO_Update(SMO_Handle *h, SMO_Params *params, 
                float Ia, float Ib, float Ualpha, float Ubeta) {
    // 1. ����������
    float e_alpha = Ia - h->Ialpha_est;
    float e_beta = Ib - h->Ibeta_est;
    
    // 2. ��ģ��������ʹ�ñ��ͺ������ٶ���
    float sat_alpha = sat(e_alpha, params->sat_boundary);
    float sat_beta = sat(e_beta, params->sat_boundary);
    
    // 3. ���綯�ƹ���
    h->EMF_alpha = params->Kslf * sat_alpha;
    h->EMF_beta = params->Kslf * sat_beta;
    
    // 4. �����۲������£���ɢ��ŷ������
    float Ts = 1.0f / params->Freq;  // ��������
    
    h->Ialpha_est += Ts * 
        ( (Ualpha - params->Rs * h->Ialpha_est - h->EMF_alpha) / params->Ls );
    
    h->Ibeta_est += Ts * 
        ( (Ubeta - params->Rs * h->Ibeta_est - h->EMF_beta) / params->Ls );
}

// ���綯�Ƶ�ͨ�˲�
void SMO_EMF_Filter(SMO_Handle *h, SMO_Params *params) {
    static float EMF_alpha_prev = 0;
    static float EMF_beta_prev = 0;
    
    // �����˲�ϵ�� �� = 2��f_cutoff * T
    float alpha = 2 * PI * params->LPF_cutoff / params->Freq;
    
    // �����˲�ϵ����0-1֮��
    if(alpha > 1.0f) alpha = 1.0f;
    if(alpha < 0.0f) alpha = 0.0f;
    
    // Ӧ��һ�׵�ͨ�˲���
    h->EMF_alpha_fil = alpha * h->EMF_alpha + (1 - alpha) * EMF_alpha_prev;
    h->EMF_beta_fil = alpha * h->EMF_beta + (1 - alpha) * EMF_beta_prev;
    
    // ���浱ǰֵ�����´��˲�
    EMF_alpha_prev = h->EMF_alpha_fil;
    EMF_beta_prev = h->EMF_beta_fil;
}

// ��ȡת��λ�ú��ٶ�
void SMO_GetPosition(SMO_Handle *h, SMO_Params *params) {
    // 1. ʹ�÷����м���ת��λ�ã���Ƕȣ�
    // ע�⣺ʹ���˲���ķ��綯��
    h->theta = fast_atan2(-h->EMF_alpha_fil, h->EMF_beta_fil);
    
    // 2. �淶���Ƕȵ�[0, 2��]
    if(h->theta < 0) 
        h->theta += 2 * PI;
    
    // 3. �ٶȹ��ƣ������ַ���
    // �������ٶȣ�rad/s��: ��_e = ���� / ��t
    float elec_omega = (h->theta - h->theta_prev) * params->Freq;
    
    // ת��Ϊ��еת�٣�RPM��: RPM = (��_e * 60) / (2�� * ������)
    h->speed = elec_omega * 60.0f / (2 * PI * params->POLES);
    
    // 4. ���½Ƕȼ���ֵ
    h->theta_prev = h->theta;
}