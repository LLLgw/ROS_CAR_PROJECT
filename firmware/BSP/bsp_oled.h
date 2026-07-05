#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include "stm32f4xx.h"
#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowString(uint8_t x,uint8_t y,const char *str);
void OLED_ShowInt(uint8_t x,uint8_t y,uint32_t num,uint8_t len);

#endif
