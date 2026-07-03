#include "state.h"
#include <string.h>

State_t g;

//全局状态初始化
void State_Init(void)
{
    memset(&g, 0, sizeof(g));				//状态清零

    g.control_source = CTRL_SOURCE_NONE;	//无控制源
    g.motion_mode = MOTION_MODE_STOP;		//停止
	g.selected_source = CTRL_SOURCE_PS2;	//默认选择手柄
    g.motor_enable = 0;						//电机转动未使能
    g.battery.voltage_mv = 25200;			//电池初始值
}
