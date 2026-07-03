#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f4xx.h"
#include <stdint.h>

#define MOTOR_PWM_MAX 9999		//PWM最大值

//四个电机枚举，提高可读性
typedef enum
{
	MOTOR_A=0,
	MOTOR_B,
	MOTOR_C,
	MOTOR_D,
	MOTOR_COUNT
} Motor_t;

void Motor_Init(void);
void Motor_SetPwm(Motor_t id,int16_t pwm);
void Motor_StopAll(void);

#endif
