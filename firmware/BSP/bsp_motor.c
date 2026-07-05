#include "bsp_motor.h"

//四个电机的方向引脚
#define MOTOR_A_IN1_PORT GPIOB
#define MOTOR_A_IN1_PIN GPIO_Pin_13
#define MOTOR_A_IN2_PORT GPIOB
#define MOTOR_A_IN2_PIN GPIO_Pin_12

#define MOTOR_B_IN1_PORT GPIOC
#define MOTOR_B_IN1_PIN GPIO_Pin_0
#define MOTOR_B_IN2_PORT GPIOE
#define MOTOR_B_IN2_PIN GPIO_Pin_15

#define MOTOR_C_IN1_PORT GPIOD
#define MOTOR_C_IN1_PIN GPIO_Pin_10
#define MOTOR_C_IN2_PORT GPIOD
#define MOTOR_C_IN2_PIN GPIO_Pin_12

#define MOTOR_D_IN1_PORT GPIOC
#define MOTOR_D_IN1_PIN GPIO_Pin_12
#define MOTOR_D_IN2_PORT GPIOA
#define MOTOR_D_IN2_PIN GPIO_Pin_8

//限幅，防止输入PWM超过最大值
static int16_t pwm_limit(int16_t pwm)
{
	if(pwm>MOTOR_PWM_MAX)
		return MOTOR_PWM_MAX;
	if(pwm<-MOTOR_PWM_MAX)
		return -MOTOR_PWM_MAX;
	return pwm;
}

//高低电平设置工具函数，代码复用
static void pin_write(GPIO_TypeDef *port, uint16_t pin, uint8_t value)
{
    if (value)
        GPIO_SetBits(port, pin);
    else
        GPIO_ResetBits(port, pin);
}

//根据PWM值设置电机转向
static void motor_dir(Motor_t id, int16_t pwm)
{
    uint8_t forward;

    if (pwm == 0)
    {
        switch (id)
        {
            case MOTOR_A:
                pin_write(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, 0);
                pin_write(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, 0);
                break;
            case MOTOR_B:
                pin_write(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, 0);
                pin_write(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, 0);
                break;
            case MOTOR_C:
                pin_write(MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN, 0);
                pin_write(MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN, 0);
                break;
            case MOTOR_D:
                pin_write(MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN, 0);
                pin_write(MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN, 0);
                break;
            default:
                break;
        }
        return;
    }

    forward = (pwm > 0) ? 1 : 0;

    switch (id)
    {
        case MOTOR_A:
            pin_write(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, forward);
            pin_write(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, !forward);
            break;
        case MOTOR_B:
            pin_write(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, forward);
            pin_write(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, !forward);
            break;
        case MOTOR_C:
            pin_write(MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN, forward);
            pin_write(MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN, !forward);
            break;
        case MOTOR_D:
            pin_write(MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN, forward);
            pin_write(MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN, !forward);
            break;
        default:
            break;
    }
}

//取绝对值工具函数
static uint16_t pwm_abs(int16_t pwm)
{
	if(pwm<0)
		return (uint16_t)(-pwm);
	return (uint16_t)pwm;
}

//通过改变PWM占空比设置电机转速
static void motor_set_compare(Motor_t id,uint16_t value)
{
	switch(id)
	{
		case MOTOR_A:
			TIM_SetCompare4(TIM8,value);
			break;
		case MOTOR_B:
			TIM_SetCompare3(TIM8,value);
			break;
		case MOTOR_C:
			TIM_SetCompare2(TIM8,value);
			break;
		case MOTOR_D:
			TIM_SetCompare1(TIM8,value);
			break;
		default:
			break;
	}
}

//定时器TIM8的PWM输出引脚及电机方向引脚初始化
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
						   RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitTypeStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitTypeStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_Init(GPIOA,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOB,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_12;
	GPIO_Init(GPIOC,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOD,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitTypeStructure);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=MOTOR_PWM_MAX;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM8,&TIM_OCInitStructure);
    TIM_OC2Init(TIM8,&TIM_OCInitStructure);
    TIM_OC3Init(TIM8,&TIM_OCInitStructure);
    TIM_OC4Init(TIM8,&TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8,TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM8,ENABLE);

    TIM_CtrlPWMOutputs(TIM8,ENABLE);
    TIM_Cmd(TIM8,ENABLE);

    Motor_StopAll();
}

//供外部调用，设置电机转向、转速
void Motor_SetPwm(Motor_t id,int16_t pwm)
{
	int16_t out;

    if (id >= MOTOR_COUNT)
        return;

    out = pwm_limit(pwm);
    motor_dir(id, out);
    motor_set_compare(id, pwm_abs(out));
}

//底盘电机停转
void Motor_StopAll(void)
{
	Motor_SetPwm(MOTOR_A, 0);
    Motor_SetPwm(MOTOR_B, 0);
    Motor_SetPwm(MOTOR_C, 0);
    Motor_SetPwm(MOTOR_D, 0);
}
