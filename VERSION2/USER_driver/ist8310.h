/****************************(C) COPYRIGHT 2021 AFShk***************************
  ==============================================================================
  * @file       ist8310.c/h
  * @brief      IST8310磁力计驱动。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021/8/27       AFShk           finish
  * 在自动分拣项目中，未使用磁力计，若需使用磁力计，只需要调用ist8310_init(),
	* ist8310_read_mag(fp32 mag[3])。
  ==============================================================================
*****************************(C) COPYRIGHT 2021 AFShk**************************/
#ifndef IST8310_H
#define IST8310_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(fp32 mag[3]);
#endif

