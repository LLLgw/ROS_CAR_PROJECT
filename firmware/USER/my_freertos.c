#include "FreeRTOS.h"
#include "task.h"


#include "my_freertos.h"
#include "state.h"
#include "drive_task.h"
#include "ros_comm_task.h"
#include "ps2_task.h"
#include "battery_task.h"
#include "display_task.h"

//调度入口前置声明
static void Control_Task(void *argument);
static void RosComm_Task(void *argument);
static void PS2_Task(void *argument);
static void Battery_Task(void *argument);
static void Display_Task(void *argument);

//创建任务
void FreeRTOS_Create(void)
{
	State_Init();											//初始化全局状态

	//任务创建配置
	xTaskCreate(Control_Task,"control",256,0,4,0);
	xTaskCreate(RosComm_Task,"ros",256,0,3,0);
	xTaskCreate(PS2_Task,"ps2",384,0,3,0);
	xTaskCreate(Battery_Task, "battery", 256, 0, 2, 0);
    xTaskCreate(Display_Task, "display", 384, 0, 1, 0);
}

//电机控制任务
static void Control_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();				//记录当前嘀嗒计数值

    while (1)
    {
        Drive_TaskUpdate();									//执行底盘驱动任务主函数

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));		//绝对延时10ms
    }
}

//ros通讯任务
static void RosComm_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();				//记录当前嘀嗒计数值

    while (1)
    {
        RosComm_TaskUpdate();								//执行ros上位机速度通讯任务主函数

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));		//绝对延时5ms
    }
}

//ps2通讯任务
static void PS2_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();				//记录当前嘀嗒计数值

    while (1)
    {
        PS2_TaskUpdate();									//执行ps2手柄速度通讯任务主函数

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));		//绝对延时1ms
    }
}

//电池电压采集任务
static void Battery_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();				//记录当前嘀嗒计数值

    while (1)
    {
        Battery_TaskUpdate();								//执行电池电压采集任务主函数

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));	//绝对延时100ms
    }
}

//显示屏显示任务
static void Display_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();				//记录当前嘀嗒计数值

    while (1)
    {
        Display_TaskUpdate();								//执行显示屏显示任务主函数

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));	//绝对延时200ms
    }
}
