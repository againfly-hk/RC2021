#ifndef USER_TASK_H
#define USER_TASK_H

#include "main.h"

extern void line_detect_task(void const * argument);
extern void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);


#endif
