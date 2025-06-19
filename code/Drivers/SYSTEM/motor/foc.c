#include "foc.h"
#include "math.h"
#include "atim.h"

Motor_t my_motor;

void foc_init(void)
{
	//参数初始化
	my_motor.params.Poles=7;
	my_motor.params.KV=220;
	my_motor.params.Maxcurrent=20;//最大电流
	my_motor.params.Maxvoltage=5.1;//最大电压
	my_motor.params.Phaseresitance=2.3;//相电阻
	my_motor.params.Wireresistance=5.1;//线电阻
	my_motor.params.WireLs=2.8;//线电感
	my_motor.params.PhaseLs=0.86;//相电感	
	my_motor.params.Kslf=0.01;//滑模增益	
	my_motor.params.Freq=10000;//PWM频率
	my_motor.params.LPF_cutoff=1000;//低通滤波器截止频率
	my_motor.params.sat_boundary=10;//饱和函数边界值
	my_motor.params.Ls=my_motor.params.PhaseLs;//定子电感
	my_motor.params.Rs=my_motor.params.Phaseresitance;//定子电阻

	//PID参数初始化	
	my_motor.pid.kp=0.1;//比例系数
	my_motor.pid.ki=0.01;//积分系数	
	my_motor.pid.integral=0;//积分值
	my_motor.pid.output_limit=100;//输出限幅	
	
	my_motor.control.maxspeed=MAXSPEED;
	my_motor.control.targetspeed=1;
	my_motor.control.voltage0.vq=Vref/2;
	//电机状态初始化
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
	
// Park逆变换：两相旋转（dq）→两相静止（αβ）[theta单位为弧度]
void inverse_park_transform(float d, float q, float theta, float *alpha, float *beta) 
{
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta  = d * sin_theta + q * cos_theta;
}

// Clarke逆变换：两相静止（αβ）→三相静止（abc）（假设零序分量为0）
void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c) 
{
    *a = alpha+Vrefby2;
    *b = -0.5 * alpha + SQRT3_BY_2 * beta+Vrefby2;
    *c = -0.5 * alpha - SQRT3_BY_2 * beta+Vrefby2;
}

// Clarke逆变换：两相静止（αβ）→三相静止（abc）（加入三次谐波注入）
void inverse_clarke_transform_with_3rd_harmonic(float alpha, float beta, float *a, float *b, float *c) 
{
    // 计算基波分量
    float base_a = alpha;
    float base_b = -0.5 * alpha + SQRT3_BY_2 * beta;
    float base_c = -0.5 * alpha - SQRT3_BY_2 * beta;
    
    // 计算三次谐波分量（作为零序分量）
    float third_harmonic = (1.0f/6.0f) * sin(3.0f * my_motor.control.angle_el);
    
    // 叠加三次谐波和PWM偏移
    *a = base_a + third_harmonic + Vrefby2;
    *b = base_b + third_harmonic + Vrefby2;
    *c = base_c + third_harmonic + Vrefby2;
}

// Clarke逆变换：两相静止（αβ）→三相静止（abc）（加入三次谐波注入）
void inverse_clarke_transform_with_3rd_harmonic_ai(float alpha, float beta, float *a, float *b, float *c) 
{
    // 计算基波分量
    float base_a = alpha;
    float base_b = -0.5 * alpha + SQRT3_BY_2 * beta;
    float base_c = -0.5 * alpha - SQRT3_BY_2 * beta;

    // 计算三次谐波分量（动态调整幅值）
    float harmonic_amplitude = 0.38 * sqrt(pow(base_a, 2) + pow(base_b, 2) + pow(base_c, 2));  //基于基波幅值计算谐波幅值
    float third_harmonic = harmonic_amplitude * sin(3.0f * my_motor.control.angle_el + PI);  //相位偏移180度

    // 叠加三次谐波和PWM偏移
    *a = base_a + third_harmonic + Vrefby2;
    *b = base_b + third_harmonic + Vrefby2;
    *c = base_c + third_harmonic + Vrefby2;

    // 电压限幅
    *a = _constrain(*a, 0.0f, Vref);
    *b = _constrain(*b, 0.0f, Vref);
    *c = _constrain(*c, 0.0f, Vref);
}

float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2*PI);   //取余，将角度限制在一个周期内，超出部分舍去
    return a >= 0 ? a : (a + 2*PI);  
}

void phasesetpwm(float *a, float *b, float *c,uint16_t *ccra,uint16_t *ccrb,uint16_t *ccrc)
{
	*ccra=_constrain(*a*1000/Vref, 0, 1000 );
	*ccrb=_constrain(*b*1000/Vref, 0, 1000 );
	*ccrc=_constrain(*c*1000/Vref, 0, 1000 );
	atim_timx_cplm_pwm_set(*ccra,1);
	atim_timx_cplm_pwm_set(*ccrb,2);
	atim_timx_cplm_pwm_set(*ccrc,3);
}

void foc_main_spwmq(void)
{
	speed_rampup();//速度rampup
	my_motor.control.speed_el=my_motor.control.speed*my_motor.params.Poles;//得到电气角速度
	my_motor.control.angle_el=_normalizeAngle(my_motor.control.speed_el*Ts+my_motor.control.angle_el);//得到弧度
	inverse_park_transform(0,my_motor.control.voltage0.vq,my_motor.control.angle_el,(float *)&my_motor.control.voltage1.valpha,(float *)&my_motor.control.voltage1.vbeta);
	inverse_clarke_transform(my_motor.control.voltage1.valpha,my_motor.control.voltage1.vbeta,(float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc);
	phasesetpwm((float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc,(uint16_t  *)&my_motor.control.CCR.ccra,(uint16_t  *)&my_motor.control.CCR.ccrb,(uint16_t  *)&my_motor.control.CCR.ccrc);
}


void foc_main_spwm(void)
{
	speed_rampup();//速度rampup
	my_motor.control.speed_el=my_motor.control.speed*my_motor.params.Poles;//得到电气角速度
	my_motor.control.angle_el=_normalizeAngle(my_motor.control.speed_el*Ts+my_motor.control.angle_el);//得到弧度
	inverse_park_transform(0,my_motor.control.voltage0.vq,my_motor.control.angle_el,(float *)&my_motor.control.voltage1.valpha,(float *)&my_motor.control.voltage1.vbeta);
	inverse_clarke_transform(my_motor.control.voltage1.valpha,my_motor.control.voltage1.vbeta,(float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc);
	phasesetpwm((float *)&my_motor.control.voltage2.va,(float *)&my_motor.control.voltage2.vb,(float *)&my_motor.control.voltage2.vc,(uint16_t  *)&my_motor.control.CCR.ccra,(uint16_t  *)&my_motor.control.CCR.ccrb,(uint16_t  *)&my_motor.control.CCR.ccrc);
}


// 七段法SVPWM计算
void SVPWM_7Segment_DP(float U_alpha, float U_beta,uint16_t *ccra,uint16_t *ccrb,uint16_t *ccrc) 
{
    float T1, T2, T0;
    uint8_t sector = 0;
    
    // 1. 扇区判断
    float U_ref = sqrtf(U_alpha*U_alpha + U_beta*U_beta);
    float theta = atan2f(U_beta, U_alpha);
    
    if (theta < 0) theta += 2 * PI;  // 转换到0-2π范围
    sector = (uint8_t)(theta / (PI/3));  // 每60度一个扇区
    
    // 2. 计算矢量作用时间 (标准公式)
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float k = SQRT3 * PWM_PERIOD / Vref;
    
    T1 = k * (sin_theta * U_alpha - cos_theta * U_beta);
    T2 = k * (SQRT3/2 * U_alpha + 0.5f * U_beta);
    T0 = PWM_PERIOD - T1 - T2;
    
    // 时间过调制处理
    if (T0 < 0) {
        T1 = T1 * PWM_PERIOD / (T1 + T2);
        T2 = T2 * PWM_PERIOD / (T1 + T2);
        T0 = 0;
    }
    
    // 3. 计算PWM比较值 (七段法对称分配)
    float Ta, Tb, Tc;
    float T_half0 = T0 / 2;
    
    switch (sector) {
        case 0: // Sector I: (100)->(110)->(111)
            Ta = T_half0 + T1 + T2;
            Tb = T_half0 + T2;
            Tc = T_half0;
            break;
        case 1: // Sector II: (110)->(010)->(111)
            Ta = T_half0 + T1;
            Tb = T_half0 + T1 + T2;
            Tc = T_half0;
            break;
        case 2: // Sector III: (010)->(011)->(111)
            Ta = T_half0;
            Tb = T_half0 + T1 + T2;
            Tc = T_half0 + T2;
            break;
        case 3: // Sector IV: (011)->(001)->(111)
            Ta = T_half0;
            Tb = T_half0 + T1;
            Tc = T_half0 + T1 + T2;
            break;
        case 4: // Sector V: (001)->(101)->(111)
            Ta = T_half0 + T2;
            Tb = T_half0;
            Tc = T_half0 + T1 + T2;
            break;
        case 5: // Sector VI: (101)->(100)->(111)
            Ta = T_half0 + T1 + T2;
            Tb = T_half0;
            Tc = T_half0 + T1;
            break;
        default:
            Ta = Tb = Tc = PWM_PERIOD / 2;
    }
    
    // 4. 转换为整数PWM比较值
    *ccra = (uint16_t)(PWM_PERIOD - Ta); // 注意极性
    *ccrb = (uint16_t)(PWM_PERIOD - Tb);
    *ccrc = (uint16_t)(PWM_PERIOD - Tc);
		atim_timx_cplm_pwm_set(*ccra*1000,1);
		atim_timx_cplm_pwm_set(*ccrb*1000,2);
		atim_timx_cplm_pwm_set(*ccrc*1000,3);
}


void SVPWM_7Segment_CP(float U_alpha, float U_beta, uint16_t *ccra, uint16_t *ccrb, uint16_t *ccrc)
{
    float T1, T2, T0;
    uint8_t sector = 0;
    float Ta, Tb, Tc;

    // 1. 电压矢量角度与模值
    float theta = atan2f(U_beta, U_alpha);
    if (theta < 0) theta += 2.0f * PI;  // 转到 [0, 2π)

    float Uref = sqrtf(U_alpha * U_alpha + U_beta * U_beta);

    // 2. 扇区判断（每 60° 一个扇区）
    sector = (uint8_t)(theta / (PI / 3.0f));

    // 3. 计算 T1/T2/T0（标准公式，投影到相邻空间矢量上）
    float angle_in_sector = theta - sector * (PI / 3.0f);  // 相对于扇区起始的角度

    float T_s = (float)PWM_PERIOD; // PWM周期
    float Vdc = Vref;              // 母线电压：Vref需事先赋值为实际Vdc值

    T1 = T_s * Uref / Vdc * sinf(PI / 3.0f - angle_in_sector);
    T2 = T_s * Uref / Vdc * sinf(angle_in_sector);
    T0 = T_s - T1 - T2;

    // 4. 对称七段分配，计算 Ta/Tb/Tc
    float T_half0 = T0 / 2.0f;

    switch (sector) {
        case 0:
            Ta = T_half0 + T1 + T2;
            Tb = T_half0 + T2;
            Tc = T_half0;
            break;
        case 1:
            Ta = T_half0 + T1;
            Tb = T_half0 + T1 + T2;
            Tc = T_half0;
            break;
        case 2:
            Ta = T_half0;
            Tb = T_half0 + T1 + T2;
            Tc = T_half0 + T2;
            break;
        case 3:
            Ta = T_half0;
            Tb = T_half0 + T1;
            Tc = T_half0 + T1 + T2;
            break;
        case 4:
            Ta = T_half0 + T2;
            Tb = T_half0;
            Tc = T_half0 + T1 + T2;
            break;
        case 5:
            Ta = T_half0 + T1 + T2;
            Tb = T_half0;
            Tc = T_half0 + T1;
            break;
        default:
            Ta = Tb = Tc = T_s / 2.0f;
            break;
    }

    // 5. 转换为PWM比较值（中心对齐，注意极性）
    *ccra = (uint16_t)(T_s - Ta);
    *ccrb = (uint16_t)(T_s - Tb);
    *ccrc = (uint16_t)(T_s - Tc);

    // 6. 输出到定时器（不需要再乘1000）
    atim_timx_cplm_pwm_set(*ccra, 1);
    atim_timx_cplm_pwm_set(*ccrb, 2);
    atim_timx_cplm_pwm_set(*ccrc, 3);
}


#define PWM_Ts  ((float)PWM_PERIOD)  // PWM周期（单位：定时器计数）
#define VBUS    Vref                 // 母线电压

void SVPWM_5Segment_CP(float Ualpha, float Ubeta, uint16_t *ccra, uint16_t *ccrb, uint16_t *ccrc)
{
    // 1. 电压矢量幅值限制 (线性区)
    float Uref = sqrtf(Ualpha * Ualpha + Ubeta * Ubeta);
    float Umax = VBUS / SQRT3;
    if (Uref > Umax) {
        float scale = Umax / Uref;
        Ualpha *= scale;
        Ubeta  *= scale;
        Uref = Umax;
    }

    // 2. 电压空间角度与扇区判断
    float angle = atan2f(Ubeta, Ualpha);
    if (angle < 0) angle += 2.0f * PI;

    uint8_t sector = (uint8_t)(angle / (PI / 3.0f));  // 0~5
    float theta = angle - sector * (PI / 3.0f);       // 扇区内相对角度

    // 3. T1, T2 计算（经典法）
    float T1 = SQRT3 * Uref / VBUS * sinf(PI / 3.0f - theta);
    float T2 = SQRT3 * Uref / VBUS * sinf(theta);
    float T0 = 1.0f - T1 - T2;

    // 4. 五段式分配 Ta, Tb, Tc (归一化时间)
    float Ta = 0, Tb = 0, Tc = 0;
    float T0_half = T0 / 2.0f;

    switch (sector) {
        case 0:
            Ta = T1 + T2 + T0_half;
            Tb = T2 + T0_half;
            Tc = T0_half;
            break;
        case 1:
            Ta = T1 + T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T0_half;
            break;
        case 2:
            Ta = T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T2 + T0_half;
            break;
        case 3:
            Ta = T0_half;
            Tb = T1 + T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 4:
            Ta = T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 5:
            Ta = T1 + T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T0_half;
            break;
        default:
            Ta = Tb = Tc = 0.5f;
            break;
    }

    // 5. 转换为PWM计数值 (中心对齐模式)
    *ccra = (uint16_t)(Ta * PWM_PERIOD);
    *ccrb = (uint16_t)(Tb * PWM_PERIOD);
    *ccrc = (uint16_t)(Tc * PWM_PERIOD);

    // 6. 安全边界处理
    if (*ccra > PWM_PERIOD) *ccra = PWM_PERIOD;
    if (*ccrb > PWM_PERIOD) *ccrb = PWM_PERIOD;
    if (*ccrc > PWM_PERIOD) *ccrc = PWM_PERIOD;

    // 7. 输出到PWM寄存器 (假设使用 TIMx 的三个通道)
    atim_timx_cplm_pwm_set(*ccra, 1);
    atim_timx_cplm_pwm_set(*ccrb, 2);
    atim_timx_cplm_pwm_set(*ccrc, 3);
}



void foc_main_svpwm(void)
{
	speed_rampup();//速度rampup
	my_motor.control.speed_el=my_motor.control.speed*my_motor.params.Poles;//得到电气角速度
	my_motor.control.angle_el=_normalizeAngle(my_motor.control.speed_el*Ts+my_motor.control.angle_el);//得到弧度
	inverse_park_transform(0,my_motor.control.voltage0.vq,my_motor.control.angle_el,(float *)&my_motor.control.voltage1.valpha,(float *)&my_motor.control.voltage1.vbeta);
	SVPWM_5Segment_CP(my_motor.control.voltage1.valpha,my_motor.control.voltage1.vbeta,(uint16_t  *)&my_motor.control.CCR.ccra,(uint16_t  *)&my_motor.control.CCR.ccrb,(uint16_t  *)&my_motor.control.CCR.ccrc);
}