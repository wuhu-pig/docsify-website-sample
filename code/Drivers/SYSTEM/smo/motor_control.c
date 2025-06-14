#include "motor_control.h"
#include "foc_math.h"

// ��ʼ���������
void MotorControl_Init(Motor_Control *mc) {
    mc->state = MOTOR_STOPPED;
    mc->start_time = 0;
    mc->open_loop_angle = 0.0f;
    mc->open_loop_freq = 1.0f; // ��ʼ����Ƶ�� (Hz)
    mc->open_loop_voltage = 1.0f; // ��ʼ������ѹ
    
    // ��ʼ��SMO����
    mc->smo_params.Rs = 0.5f;        // ������� (��)
    mc->smo_params.Ls = 0.001f;      // ������ (H)
    mc->smo_params.Kslf = 0.2f;      // ��ģ����
    mc->smo_params.Freq = 20000.0f;  // PWMƵ�� (Hz)
    mc->smo_params.POLES = 7;        // ���������
    mc->smo_params.LPF_cutoff = 50.0f; // ��ͨ�˲���ֹƵ�� (Hz)
    mc->smo_params.sat_boundary = 0.05f; // ���ͺ����߽�ֵ
    
    // ��ʼ��SMO
    SMO_Init(&mc->smo);
    
    // ��ʼ��PID������
    PID_Init(&mc->pid_speed, 0.1f, 0.01f, 0.001f, -10.0f, 10.0f, 1.0f); // �ٶȻ�
    PID_Init(&mc->pid_id, 1.0f, 0.1f, 0.0f, -5.0f, 5.0f, 1.0f);         // d�������
    PID_Init(&mc->pid_iq, 1.0f, 0.1f, 0.0f, -5.0f, 5.0f, 1.0f);         // q�������
}

// ���������ѭ��
void MotorControl_Update(Motor_Control *mc, 
                         float Ia, float Ib, float Ic, 
                         float Vdc, uint32_t timestamp) {
    float Ialpha, Ibeta;
    float Id, Iq;
    float Valpha, Vbeta;
    float Ualpha, Ubeta; // ʵ��ʩ�ӵĵ�ѹ
    
    // 1. Clarke�任
    Clarke_Transform(Ia, Ib, Ic, &Ialpha, &Ibeta);
    
    switch(mc->state) {
        case MOTOR_STOPPED:
            // ֹͣ״̬�������PWM
            // ... (����PWM���Ϊ��)
            break;
            
        case MOTOR_ALIGNMENT:
            // Ԥ��λ�׶Σ��̶��Ƕȣ�
            mc->smo.theta = 0.0f; // �̶�0��λ��
            
            // ���ù̶���ѹ
            Valpha = mc->open_loop_voltage;
            Vbeta = 0.0f;
            
            // ����Ƿ����Ԥ��λ
            if(timestamp - mc->start_time > 500) { // 500ms�����Ԥ��λ
                mc->state = MOTOR_OPEN_LOOP;
                mc->start_time = timestamp;
            }
            break;
            
        case MOTOR_OPEN_LOOP:
            // ���������׶Σ�VF���ƣ�
            mc->open_loop_angle += mc->open_loop_freq / mc->smo_params.Freq * TWO_PI;
            if(mc->open_loop_angle > TWO_PI) mc->open_loop_angle -= TWO_PI;
            
            // ���ÿ�����ѹʸ��
            Valpha = mc->open_loop_voltage * cosf(mc->open_loop_angle);
            Vbeta = mc->open_loop_voltage * sinf(mc->open_loop_angle);
            
            // ������Ƶ�ʺ͵�ѹ
            if(timestamp - mc->start_time > 10) { // ÿ10ms����һ��
                mc->open_loop_freq += 0.1f; // Ƶ������0.1Hz
                mc->open_loop_voltage += 0.01f; // ��ѹ����0.01
                mc->start_time = timestamp;
            }
            
            // ��Ƶ�ʴﵽһ��ֵʱ�л����ջ�
            if(mc->open_loop_freq > 5.0f) { // �ﵽ5Hzʱ�л�
                mc->state = MOTOR_CLOSED_LOOP;
                mc->smo.theta_prev = mc->open_loop_angle; // ��ʼ���Ƕ�
            }
            break;
            
        case MOTOR_CLOSED_LOOP:
            // �ջ����У�FOC���ƣ�
            
            // 2. Park�任
            Park_Transform(Ialpha, Ibeta, mc->smo.theta, &Id, &Iq);
            
            // 3. ����SMO�۲�������Ҫʵ��ʩ�ӵĵ�ѹ��
            SMO_Update(&mc->smo, &mc->smo_params, Ia, Ib, Ualpha, Ubeta);
            SMO_EMF_Filter(&mc->smo, &mc->smo_params);
            SMO_GetPosition(&mc->smo, &mc->smo_params);
            
            // 4. �ٶȻ�PID
            float speed_ref = 1000.0f; // Ŀ���ٶ�1000 RPM
            float Iq_ref = PID_Update(&mc->pid_speed, speed_ref, mc->smo.speed, 
                                     1.0f/mc->smo_params.Freq);
            
            // 5. ������PID
            float Id_ref = 0.0f; // d�������Ϊ0�����ת�ؿ��ƣ�
            float Vd = PID_Update(&mc->pid_id, Id_ref, Id, 1.0f/mc->smo_params.Freq);
            float Vq = PID_Update(&mc->pid_iq, Iq_ref, Iq, 1.0f/mc->smo_params.Freq);
            
            // 6. ��Park�任
            Inv_Park_Transform(Vd, Vq, mc->smo.theta, &Valpha, &Vbeta);
            break;
    }
    
    // 7. ����SVPWM
    float Ta, Tb, Tc;
    SVPWM_Generate(Valpha, Vbeta, Vdc, &Ta, &Tb, &Tc);
    
    // 8. ����PWM���������Ӳ��ʵ�֣�
    // ... (���ö�ʱ��ռ�ձ�)
    
    // ����ʵ��ʩ�ӵĵ�ѹ����SMO
    Ualpha = (Ta - 0.5f) * Vdc;
    Ubeta = (Tb - 0.5f) * Vdc * ONE_BY_SQRT3; // ���Ƽ���
}