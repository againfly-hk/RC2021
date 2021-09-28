/****************************(C) COPYRIGHT 2021 AFShk***************************
  ==============================================================================
  * @file       bsp_iic.c/h
  * @brief      IST8310������ͨ�Ų㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
                �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021/8/27       AFShk           finish
  ==============================================================================
*****************************(C) COPYRIGHT 2021 AFShk**************************/

#include "bsp_iic.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"

extern I2C_HandleTypeDef hi2c3;
uint8_t ist8310errcode;

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    ist8310errcode=HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void ist8310_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

void ist8310_delay_us(uint16_t us)
{
    delay_us(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}

void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
