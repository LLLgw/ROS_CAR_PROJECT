#ifndef __BSP_ESTOP_H
#define __BSP_ESTOP_H

#include "stm32f4xx.h"
#include <stdint.h>

void EStop_Init(void);
uint8_t EStop_IsPressed(void);

#endif
