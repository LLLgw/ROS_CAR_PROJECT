#include "display_task.h"
#include "state.h"
#include "bsp_oled.h"
#include "bsp_estop.h"

//控制源转字符串
static const char *mode(void)
{
	//全局状态控制源判断
    if (g.selected_source == CTRL_SOURCE_ROS)
    {
        return "ROS";
    }

    if (g.selected_source == CTRL_SOURCE_PS2)
    {
        return "PS2";
    }

    return "NONE";
}

//双路径低电判断
static uint8_t battery_low(void)
{
	//电池低电标志
    if ((g.battery.flags & 0x0001) != 0)
    {
        return 1;
    }

	//全局低电标志
    if (g.low_battery)
    {
        return 1;
    }

    return 0;
}

//放大1000倍显示带符号float
static void show_milli_value(uint8_t x,uint8_t y,float value)
{
	uint32_t milli;

	//确定符号
	if(value<0.0f)
	{
		OLED_ShowString(x,y,"-");
		value=-value;										//取绝对值
	}
	else
	{
		OLED_ShowString(x,y,"+");
	}

	milli=(uint32_t)(value*1000.0f);
	OLED_ShowInt((uint8_t)(x+6),y,milli,4);
}

//显示屏显示任务主函数
void Display_TaskUpdate(void)
{
	OLED_Clear();											//显存清零
	
	OLED_ShowString(0,0,"RTOS");							//显示系统
	
	//ROS在线状态
	if(g.ros_online)
		OLED_ShowString(0,1,"ROS: OK");
	else
		OLED_ShowString(0,1,"ROS: --");
	
	//PS2在线状态
	if(g.ps2_online)
		OLED_ShowString(0,2,"PS2: OK");
	else
		OLED_ShowString(0,2,"PS2: --");
	
	//显示电池电压
	OLED_ShowString(0,3,"BAT:");
	OLED_ShowInt(28,3,g.battery.voltage_mv,5);

	//低电判断
	if (battery_low())
	{
		OLED_ShowString(76,3,"LOW");					//低电显示LOW
	}
	else
	{
		OLED_ShowInt(76,3,g.battery.percent,3);			//非低电显示剩余量百分比
		OLED_ShowString(100,3,"%");
	}
	
	//控制源显示
	OLED_ShowString(0,4,"MODE:");
	OLED_ShowString(42,4,mode());
	
	//当前底盘VX显示
	OLED_ShowString(0,5,"VX:");
	show_milli_value(28,5,g.feedback_speed.vx_mps);
	
	//当前底盘WZ显示
	OLED_ShowString(0,6,"WZ:");
	show_milli_value(28,6,g.feedback_speed.wz_radps);
	
	//显示软件急停标志
	OLED_ShowString(0, 7, "ESTOP:");
	OLED_ShowInt(48, 7, EStop_IsPressed(), 1);
	
	//刷新到屏幕
	OLED_Refresh();
}
