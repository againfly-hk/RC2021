#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "struct_typedef.h"
#include "main.h"
#include "i2c.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310��IIC��ַ
#define IST8310_IIC_READ_MSB (0x80) //IST8310��SPI��ȡ���͵�һ��bitΪ1

extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //��λIO �ø�
extern void ist8310_RST_L(void); //��λIO �õ� �õػ�����ist8310����

#endif
