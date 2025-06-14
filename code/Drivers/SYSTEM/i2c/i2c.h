#ifndef __I2C_H
#define __I2C_H

#include "sys.h"

// AS5600 I2C ��ַ��7λ��ַ��ʵ��ͨ��������һλ�����R/Wλ��
#define AS5600_I2C_ADDR        0x36

// �Ĵ�����ַ
#define AS5600_ANGLE_REGISTER  0x0C

// ��ʼ���ӿ�
void AS5600_Init(void);

// ��ȡ�Ƕȣ�����ֵΪ0~4095��
uint16_t AS5600_ReadRawAngle(void);

// ��ȡ�Ƕȣ�0~360�㣩
float AS5600_ReadAngle_Degrees(void);

// ��ȡ�Ƕȣ�-��~�У�
float AS5600_ReadAngle_Radians(void);

#endif
