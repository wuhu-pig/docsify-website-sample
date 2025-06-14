#include "foc.h"
#include "math.h"
#include "atim.h"

Motor_t my_motor;

void foc_init(void)
{
	//������ʼ��
	my_motor.params.Poles=7;
	my_motor.params.KV=220;
	my_motor.params.Maxcurrent=20;//������
	my_motor.params.Maxvoltage=5.1;//����ѹ
	my_motor.params.Phaseresitance=2.3;//�����
	my_motor.params.Wireresistance=5.1;//�ߵ���
	my_motor.params.WireLs=2.8;//�ߵ��
	my_motor.params.PhaseLs=0.86;//����	
	my_motor.params.Kslf=0.01;//��ģ����	
	my_motor.params.Freq=10000;//PWMƵ��
	my_motor.params.LPF_cutoff=1000;//��ͨ�˲�����ֹƵ��
	my_motor.params.sat_boundary=10;//���ͺ����߽�ֵ
	my_motor.params.Ls=my_motor.params.PhaseLs;//���ӵ��
	my_motor.params.Rs=my_motor.params.Phaseresitance;//���ӵ���

	//PID������ʼ��	
	my_motor.pid.kp=0.1;//����ϵ��
	my_motor.pid.ki=0.01;//����ϵ��	
	my_motor.pid.integral=0;//����ֵ
	my_motor.pid.output_limit=100;//����޷�	
	
	my_motor.control.maxspeed=MAXSPEED;
	my_motor.control.targetspeed=1;
	my_motor.control.voltage0.vq=Vref/2;
	//���״̬��ʼ��
	my_motor.state=MOTOR_STOPPED;
}
void speed_rampup(void)
{
	if(my_motor.control.speed<=my_motor.control.maxspeed)
	{
		if(my_motor.control.speed<my_motor.control.targetspeed)
			{
				my_motor.control.speed+=RAMPSTEP;
			}
	}
}
	
// Park��任��������ת��dq�������ྲֹ�����£�[theta��λΪ����]
void inverse_park_transform(float d, float q, float theta, float *alpha, float *beta) 
{
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta  = d * sin_theta + q * cos_theta;
}

// Clarke��任�����ྲֹ�����£������ྲֹ��abc���������������Ϊ0��
void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c) 
{
    *a = alpha+Vrefby2;
    *b = -0.5 * alpha + SQRT3_BY_2 * beta+Vrefby2;
    *c = -0.5 * alpha - SQRT3_BY_2 * beta+Vrefby2;
}

float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2*PI);   //ȡ�࣬���Ƕ�������һ�������ڣ�����������ȥ
    return a >= 0 ? a : (a + 2*PI);  
}

void phasesetpwm(float *a, float *b, float *c,uint16_t *ccra,uint16_t *ccrb,uint16_t *ccrc)
{
	*ccra=_constrain(*a*1000/Vref, 0, 1000 );
	*ccrb=_constrain(*b*1000/Vref, 0, 1000 );
	*ccrc=_constrain(*c*1000/Vref, 0, 1000 );
	atim_timx_cplm_pwm_set(*ccra*1000,1);
	atim_timx_cplm_pwm_set(*ccrb*1000,2);
	atim_timx_cplm_pwm_set(*ccrc*1000,3);
}
void foc_main(void)
{
	speed_rampup();//�ٶ�rampup
	my_motor.control.speed_el=my_motor.control.speed*my_motor.params.Poles;//�õ��������ٶ�
	my_motor.control.angle_el=_normalizeAngle(my_motor.control.speed_el*Ts+my_motor.control.angle_el);//�õ�����
	inverse_park_transform(0,my_motor.control.voltage0.vq,my_motor.control.angle_el,(float *)&my_motor.control.voltage1.valpha,(float *)&my_motor.control.voltage1.vbeta);
	inverse_clarke_transform(my_motor.control.voltage1.valpha,my_motor.control.voltage1.vbeta,(float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc);
	phasesetpwm((float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc,(uint16_t  *)&my_motor.control.CCR.ccra,(uint16_t  *)&my_motor.control.CCR.ccrb,(uint16_t  *)&my_motor.control.CCR.ccrc);
}