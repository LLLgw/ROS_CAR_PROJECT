#include "ps2_task.h"
#include "state.h"
#include "bsp_usb.h"
#include "FreeRTOS.h"
#include "task.h"

#define PS2_DEAD_ZONE          20							//摇杆死区
#define PS2_SPEED_MIN_MPS      0.10f						//最低速度
#define PS2_SPEED_MAX_MPS      0.45f						//最高速度
#define PS2_SPEED_STEP_MPS     0.05f						//加减速步长
#define PS2_TURN_RADIUS_M      0.453f						//自转时轮子与旋转中心的距离

static uint8_t ps2_unlocked = 0;							//手柄锁标志
static uint16_t last_buttons = 0;							//上一次单击的按键状态
static float speed_limit_mps = 0.25f;						//默认速度

//死区过滤，防手抖，稳回中
static int16_t apply_deadzone(int16_t value)
{
	//摇杆死区内不返回速度指令
    if ((value > -PS2_DEAD_ZONE) && (value < PS2_DEAD_ZONE))
    {
        return 0;
    }

    return value;
}

//单击检测
static uint8_t button_pressed_once(uint16_t now, uint16_t mask)
{
    return ((now & mask) != 0) && ((last_buttons & mask) == 0);	//上一帧没按这一帧按了
}

//ps2指令清零
static void stop_ps2_cmd(void)
{
    g.ps2_cmd.online = 0;
    g.ps2_cmd.vx_mps = 0.0f;
    g.ps2_cmd.vy_mps = 0.0f;
    g.ps2_cmd.wz_radps = 0.0f;
}

//读取手柄状态主务主函数
void PS2_TaskUpdate(void)
{
    UsbPadState_t pad;
    int16_t forward;
    int16_t turn;
    float vx;
    float wz;

    Usb_Update();												//运行USB库

    g.ps2_online = Usb_GetState(&pad);							//获取手柄数据
	
    if (!g.ps2_online)
	{
		ps2_unlocked = 0;
		last_buttons = 0;
		stop_ps2_cmd();
		return;
	}

	//单击解锁键开锁
    if (button_pressed_once(pad.buttons, USBPAD_BTN_START))
    {
        ps2_unlocked = 1;
    }
	
	//单击x键换控制模式
	if (button_pressed_once(pad.buttons, USBPAD_BTN_X))
	{
		if (g.selected_source == CTRL_SOURCE_PS2)
		{
			g.selected_source = CTRL_SOURCE_ROS;
		}
		else
		{
			g.selected_source = CTRL_SOURCE_PS2;
		}

		stop_ps2_cmd();
	}

	//单击LB键减速
    if (button_pressed_once(pad.buttons, USBPAD_BTN_LB))
    {
        speed_limit_mps -= PS2_SPEED_STEP_MPS;
        if (speed_limit_mps < PS2_SPEED_MIN_MPS)
        {
            speed_limit_mps = PS2_SPEED_MIN_MPS;					//不低于0.1
        }
    }

	//单击RB键加速
    if (button_pressed_once(pad.buttons, USBPAD_BTN_RB))
    {
        speed_limit_mps += PS2_SPEED_STEP_MPS;
        if (speed_limit_mps > PS2_SPEED_MAX_MPS)
        {
            speed_limit_mps = PS2_SPEED_MAX_MPS;					//不大于0.45
        }
    }

    last_buttons = pad.buttons;

	//未解锁
    if (!ps2_unlocked)
    {
        stop_ps2_cmd();												//ps2指令清零
        return;
    }

    forward = apply_deadzone(pad.lx);								//左摇杆x轴控制前后
    turn = apply_deadzone(pad.ry);									//右摇杆y轴控制旋转

	//左摇杆过死区
    if (forward != 0)
    {
        vx = ((float)forward / 127.0f) * speed_limit_mps;			
    }
	//只按右扳机
    else if ((pad.rt != 0) && (pad.lt == 0))
    {
        vx = ((float)pad.rt / 255.0f) * speed_limit_mps;
    }
	//只按左扳机
    else if ((pad.lt != 0) && (pad.rt == 0))
    {
        vx = -((float)pad.lt / 255.0f) * speed_limit_mps;			//负号后退
    }
    else
    {
        vx = 0.0f;
    }

    wz = ((float)turn / 127.0f) * (speed_limit_mps / PS2_TURN_RADIUS_M);

	//写入全局指令
    g.ps2_cmd.vx_mps = vx;
    g.ps2_cmd.vy_mps = 0.0f;
    g.ps2_cmd.wz_radps = wz;
    g.ps2_cmd.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g.ps2_cmd.online = 1;
}
