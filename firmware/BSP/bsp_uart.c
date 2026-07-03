#include "bsp_uart.h"
#include "ros_frame.h"

//uart3引脚初始化、中断初始化
void UART3_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate=baud;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;				//同时收发
	USART_Init(USART3,&USART_InitStructure);
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);								//中断使能
	
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;						//抢占式优先级5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;							//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3,ENABLE);
}

//发送字节
void UART3_SendByte(uint8_t data)
{
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
	USART_SendData(USART3,data);
}

//发送数组
void UART3_SendArr(uint8_t *buf,uint16_t len)
{
	uint16_t i;
	for(i=0;i<len;i++)
		UART3_SendByte(buf[i]);
}

//中断
void USART3_IRQHandler(void)
{
	uint8_t data;
	
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{
		data=USART_ReceiveData(USART3);
		
		RosFrame_InputByte(data);						//协议层解析
		
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);	//清中断
	}
}
