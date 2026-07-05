#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx.h"
#include <stdint.h>

void UART3_Init(uint32_t baud);
void UART3_SendByte(uint8_t data);
void UART3_SendArr(uint8_t *buf,uint16_t len);

#endif
