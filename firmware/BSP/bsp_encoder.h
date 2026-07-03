#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f4xx.h"
#include "stdint.h"

//四个编码器枚举，提高可读性
typedef enum
{
	ENCODER_A=0,
	ENCODER_B,
	ENCODER_C,
	ENCODER_D,
	ENCODER_COUNT
} Encoder_t;

void Encoder_Init(void);
int16_t Encoder_ReadDelta(Encoder_t id);

#endif
