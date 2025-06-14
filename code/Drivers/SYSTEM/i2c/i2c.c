#include "i2c.h"
#include "delay.h"

#define I2C_DIRECTION_TRANSMITTER  0x00
#define I2C_DIRECTION_RECEIVER     0x01
#define I2C_TIMEOUT                10000

/***************************************************************
 * �ڲ��������ȴ� I2C ��־λ
 ***************************************************************/
static uint8_t I2C2_WaitFlag(volatile uint32_t* reg, uint32_t flag, uint8_t set) {
    uint32_t timeout = I2C_TIMEOUT;
    if (set) {
        while (((*reg) & flag) == 0) {
            if (--timeout == 0) return 0;
        }
    } else {
        while (((*reg) & flag) != 0) {
            if (--timeout == 0) return 0;
        }
    }
    return 1;
}

/***************************************************************
 * ���� START �źŲ����͵�ַ
 ***************************************************************/
void I2C2_Start(uint8_t address, uint8_t direction) {
    if (!I2C2_WaitFlag(&I2C2->SR2, I2C_SR2_BUSY, 0)) return;

    I2C2->CR1 |= I2C_CR1_START;
    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_SB, 1)) return;

    (void)I2C2->SR1; // ���SB
    I2C2->DR = (address << 1) | direction;

    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_ADDR, 1)) return;
    (void)I2C2->SR1;
    (void)I2C2->SR2;
}

/***************************************************************
 * д��һ���ֽ�����
 ***************************************************************/
void I2C2_Write(uint8_t data) {
    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_TXE, 1)) return;
    I2C2->DR = data;
    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_BTF, 1)) return;
}

/***************************************************************
 * ��ȡһ���ֽڲ����� ACK
 ***************************************************************/
uint8_t I2C2_Read_ACK(void) {
    I2C2->CR1 |= I2C_CR1_ACK;
    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_RXNE, 1)) return 0xFF;
    return I2C2->DR;
}

/***************************************************************
 * ��ȡһ���ֽڲ����� NACK + STOP
 ***************************************************************/
uint8_t I2C2_Read_NACK(void) {
    uint8_t data;
    I2C2->CR1 &= ~I2C_CR1_ACK;
    if (!I2C2_WaitFlag(&I2C2->SR1, I2C_SR1_RXNE, 1)) return 0xFF;
    data = I2C2->DR;
    I2C2->CR1 |= I2C_CR1_STOP;
    return data;
}

/***************************************************************
 * ���� STOP �ź�
 ***************************************************************/
void I2C2_Stop(void) {
    I2C2->CR1 |= I2C_CR1_STOP;
    delay_us(10); // ���ӻ�һ��ֹͣ����ʱ��
}

/***************************************************************
 * I2C2 ��ʼ����ʹ�� PF0=SDA��PF1=SCL��
 ***************************************************************/
void AS5600_Init(void) {
    // ����ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // ���� PF0 �� PF1 Ϊ���ù��� AF4��I2C2��
    GPIOF->MODER   |= (2 << (0 * 2)) | (2 << (1 * 2));  // ���ù���
    GPIOF->OTYPER  |= (1 << 0) | (1 << 1);              // ��©���
    GPIOF->OSPEEDR |= (3 << (0 * 2)) | (3 << (1 * 2));  // ����
    GPIOF->PUPDR   |= (1 << (0 * 2)) | (1 << (1 * 2));  // ����
    GPIOF->AFR[0]  |= (4 << (0 * 4)) | (4 << (1 * 4));  // AF4 = I2C2

    // I2C ����
    I2C2->CR1 &= ~I2C_CR1_PE;   // ��ֹ I2C
    I2C2->CR2 = 42;             // ����ʱ�ӣ�MHz��������Ƶһ��
    I2C2->CCR = 210;            // ��׼ģʽ��100kHz��
    I2C2->TRISE = 43;           // �������ʱ��
    I2C2->CR1 |= I2C_CR1_PE;    // ʹ�� I2C
}

/***************************************************************
 * ��ȡ AS5600 ԭʼ�Ƕȣ����� 0~4095��
 ***************************************************************/
uint16_t AS5600_ReadRawAngle(void) {
    uint8_t high, low;
    uint16_t angle;

    // ��һ����д�Ĵ�����ַ
    I2C2_Start(AS5600_I2C_ADDR, I2C_DIRECTION_TRANSMITTER);
    I2C2_Write(AS5600_ANGLE_REGISTER);
    I2C2_Stop();
    delay_us(10);  // �� AS5600 ����ʱ��

    // �ڶ�������ȡ�ߵ��ֽ�
    I2C2_Start(AS5600_I2C_ADDR, I2C_DIRECTION_RECEIVER);
    high = I2C2_Read_ACK();
    low  = I2C2_Read_NACK();

    if (high == 0xFF && low == 0xFF) return 0xFFFF;  // ͨ��ʧ��
    angle = ((uint16_t)high << 8) | low;
    return angle & 0x0FFF;  // ������ 12 λ
}

/***************************************************************
 * ��ȡ�Ƕȣ�ת��Ϊ 0~360.0�㣩
 ***************************************************************/
float AS5600_ReadAngle_Degrees(void) {
    uint16_t raw = AS5600_ReadRawAngle();
    if (raw == 0xFFFF) return -1.0f;  // �����־
    return (float)raw * 360.0f / 4096.0f;
}

/***************************************************************
 * ��ȡ�Ƕȣ�ת��Ϊ -��~�У�
 ***************************************************************/
float AS5600_ReadAngle_Radians(void) {
    uint16_t raw = AS5600_ReadRawAngle();
    if (raw == 0xFFFF) return -10.0f;  // �����־
    float radians = ((float)raw / 4096.0f) * (2.0f * 3.1415926f);
    if (radians > 3.1415926f)
        radians -= 2.0f * 3.1415926f;
    return radians;
}


#define OLED_I2C_ADDR 0x3C  // SSD1306 7λ��ַ��0x3C��0x3D���Ӿ���Ӳ����

// I2C1 ��ʼ����PB6=SCL, PB7=SDA��
void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // ʹ��GPIOBʱ��
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // ʹ��I2C1ʱ��

    // ����PB6��PB7Ϊ���ù��� AF4 (I2C1)
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |= (2 << (6 * 2)) | (2 << (7 * 2));   // ����ģʽ
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);              // ��©���
    GPIOB->OSPEEDR |= (3 << (6 * 2)) | (3 << (7 * 2)); // ����
    GPIOB->PUPDR |= (1 << (6 * 2)) | (1 << (7 * 2));   // ����
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4));  // AF4 = I2C1

    I2C1->CR1 &= ~I2C_CR1_PE;        // �ر�I2C1
    I2C1->CR2 = 42;                  // APB1ʱ��42MHz
    I2C1->CCR = 210;                 // ��׼ģʽ 100kHz (42MHz/210/2=100kHz)
    I2C1->TRISE = 43;                // TRISE = freq + 1
    I2C1->CR1 |= I2C_CR1_PE;         // ʹ��I2C1
}

// �ȴ���־λ������set=1�ȴ���־λ��λ��set=0�ȴ���־λ����
static uint8_t I2C1_WaitFlag(volatile uint32_t* reg, uint32_t flag, uint8_t set) {
    uint32_t timeout = 10000;
    if (set) {
        while (((*reg) & flag) == 0) {
            if (--timeout == 0) return 0;
        }
    } else {
        while (((*reg) & flag) != 0) {
            if (--timeout == 0) return 0;
        }
    }
    return 1;
}

// I2C1 ������ʼ�źŲ������豸��ַ+��дλ
uint8_t I2C1_Start(uint8_t address, uint8_t direction) {
    if (!I2C1_WaitFlag(&I2C1->SR2, I2C_SR2_BUSY, 0)) return 0;  // �ȴ����߿���

    I2C1->CR1 |= I2C_CR1_START;                                // ����START
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_SB, 1)) return 0;  // �ȴ�SB��λ

    (void)I2C1->SR1;                                          // ��SR1���SB��־
    I2C1->DR = (address << 1) | direction;                    // ���͵�ַ

    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR, 1)) return 0; // �ȴ�ADDR��λ
    (void)I2C1->SR1;                                          // ��SR1���ADDR��־
    (void)I2C1->SR2;                                          // ��SR2��ɵ�ַ�׶�
    return 1;
}

// I2C1 ���������ֽ�
uint8_t I2C1_Write(uint8_t data) {
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1)) return 0;  // �ȴ�TXE��λ
    I2C1->DR = data;                                          // ��������
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_BTF, 1)) return 0; // �ȴ�BTF��λ
    return 1;
}

// I2C1 ����ֹͣ�ź�
void I2C1_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    delay_us(10);
}

// OLED д����
void OLED_WriteCommand(uint8_t cmd) {
    I2C1_Start(OLED_I2C_ADDR, 0);       // дģʽ
    I2C1_Write(0x00);                   // �����ֽ� 0x00 ��ʾ����
    I2C1_Write(cmd);
    I2C1_Stop();
}

// OLED д����
void OLED_WriteData(uint8_t data) {
    I2C1_Start(OLED_I2C_ADDR, 0);       // дģʽ
    I2C1_Write(0x40);                   // �����ֽ� 0x40 ��ʾ����
    I2C1_Write(data);
    I2C1_Stop();
}

// OLED ��ʼ����SSD1306ʾ����
void OLED_Init(void) {
    delay_ms(100);      // �ϵ���ʱ
    OLED_WriteCommand(0xAE); // �ر���ʾ
    OLED_WriteCommand(0x20); // �����ڴ��ַģʽ
    OLED_WriteCommand(0x10); // ҳ��ַģʽ
    OLED_WriteCommand(0xB0); // ����ҳ��ʼ��ַ
    OLED_WriteCommand(0xC8); // COM���ɨ�跽��ת
    OLED_WriteCommand(0x00); // ���õ��е�ַ
    OLED_WriteCommand(0x10); // ���ø��е�ַ
    OLED_WriteCommand(0x40); // ������ʼ�е�ַ
    OLED_WriteCommand(0x81); // ���öԱȶ�
    OLED_WriteCommand(0xFF); // ���Աȶ�
    OLED_WriteCommand(0xA1); // ����ӳ��
    OLED_WriteCommand(0xA6); // ������ʾ
    OLED_WriteCommand(0xA8); // ��·���ñ���
    OLED_WriteCommand(0x3F); // 1/64
    OLED_WriteCommand(0xA4); // ���������RAM����
    OLED_WriteCommand(0xD3); // ������ʾƫ��
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xD5); // ����ʱ�ӷ�Ƶ
    OLED_WriteCommand(0xF0);
    OLED_WriteCommand(0xD9); // ����Ԥ�������
    OLED_WriteCommand(0x22);
    OLED_WriteCommand(0xDA); // ����COM����Ӳ������
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB); // ����VCOMH��ѹ����
    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x8D); // ��������
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF); // ����ʾ
}

// OLED ����
void OLED_Clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        OLED_WriteCommand(0xB0 + page); // ����ҳ��ַ
        OLED_WriteCommand(0x00);        // ���õ��е�ַ
        OLED_WriteCommand(0x10);        // ���ø��е�ַ
        for (uint8_t col = 0; col < 128; col++) {
            OLED_WriteData(0x00);       // ��������
        }
    }
}

// ����ʾ��������OLED��0ҳдһ���ֽڣ���ʾ��
void OLED_ShowByte(uint8_t page, uint8_t col, uint8_t data) {
    OLED_WriteCommand(0xB0 + page);       // ҳ��ַ
    OLED_WriteCommand(0x00 + (col & 0x0F));    // ���е�ַ
    OLED_WriteCommand(0x10 + ((col >> 4) & 0x0F));  // ���е�ַ
    OLED_WriteData(data);
}
