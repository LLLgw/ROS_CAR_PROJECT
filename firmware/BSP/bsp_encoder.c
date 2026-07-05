#include "bsp_encoder.h"

//定时器TIM2/3/4/5的编码器引脚初始化
void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
						   RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 |
								GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,
				TIM_EncoderMode_TI12,		//双通道计数500相提高为2000相
				TIM_ICPolarity_Rising,
				TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM3,
				TIM_EncoderMode_TI12,
				TIM_ICPolarity_Rising,
				TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4,
				TIM_EncoderMode_TI12,
				TIM_ICPolarity_Rising,
				TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM5,
				TIM_EncoderMode_TI12,
				TIM_ICPolarity_Rising,
				TIM_ICPolarity_Rising);
				
	TIM_SetCounter(TIM2,0);
	TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);
	TIM_SetCounter(TIM5,0);
	
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
	TIM_Cmd(TIM5,ENABLE);		
}

//读取转数增量
int16_t Encoder_ReadDelta(Encoder_t id)
{
	int16_t delta = 0;

    if(id == ENCODER_A)
    {
        delta = (int16_t)TIM_GetCounter(TIM2);		//转为int16_t，符号表示转向
        TIM_SetCounter(TIM2, 0);					//读完清0，只读增量
    }
    else if(id == ENCODER_B)
    {
        delta = (int16_t)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);
    }
    else if(id == ENCODER_C)
    {
        delta = (int16_t)TIM_GetCounter(TIM4);
        TIM_SetCounter(TIM4, 0);
    }
    else if(id == ENCODER_D)
    {
        delta = (int16_t)TIM_GetCounter(TIM5);
        TIM_SetCounter(TIM5, 0);
    }

    return delta;
}
