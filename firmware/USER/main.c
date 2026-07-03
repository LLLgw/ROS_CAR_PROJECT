#include "FreeRTOS.h"
#include "task.h"

#include "Board_Init.h"
#include "my_freertos.h"

int main(void)
{
	Board_Init();				//板级硬件初始化
	
	FreeRTOS_Create();			//创建任务
	
	vTaskStartScheduler();		//启动任务调度器
	
	while(1)
	{
		
	}
}
