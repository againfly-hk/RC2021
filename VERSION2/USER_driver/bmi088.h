/****************************(C) COPYRIGHT 2021 AFShk***************************
  ==============================================================================
  * @file       bmi088.c/h
  * @brief      BMI088Çý¶¯¡£
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021/8/27       AFShk           finish
  ==============================================================================
*****************************(C) COPYRIGHT 2021 AFShk**************************/
#ifndef BMI088_H
#define BMI088_H

#include "struct_typedef.h"

#define BMI088_USE_SPI

extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
