#include "Board_Init.h"
#include "delay.h"
#include "bsp_oled.h"
#include "bsp_uart.h"
#include "ros_frame.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_usb.h"
#include "battery_task.h"
#include "bsp_estop.h"

void Board_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);			//中断优先级分组
	
	Delay_Init();											//DWT延时初始化
	
	OLED_Init();											//OLED显示屏初始化
	
	UART3_Init(115200);										//配置串口3波特率
	RosFrame_Init();										//协议初始化
	
	Motor_Init();											//电机PWM初始化
	Encoder_Init();											//编码器初始化
	
	Battery_Init();											//电池ADC初始化
	
	Usb_Init();												//USB初始化
	
	EStop_Init();
			
	Motor_StopAll();										//底盘电机停转
}
