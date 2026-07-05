#include "FreeRTOS.h"
#include "task.h"

#include "ros_comm_task.h"
#include "ros_frame.h"
#include "state.h"
#include "bsp_uart.h"

#define ROS_TIMEOUT_MS        500					//ros下发帧超时间隔
#define ROS_UPLOAD_PERIOD_MS  10					//底盘速度上报帧间隔
#define ROS_BAT_UPLOAD_PERIOD_MS 100				//电池上报帧间隔

static uint32_t s_last_rx_ms = 0;					//最后一次接收ros帧时间
static uint32_t s_last_tx_ms = 0;					//最后一次上报底盘速度帧时间
static uint32_t s_last_bat_tx_ms = 0;				//最后一次上报电池帧时间

//ROS速度通讯任务主函数
void RosComm_TaskUpdate(void)
{
    RosCmd_t cmd;
    uint32_t now;

    now = xTaskGetTickCount() * portTICK_PERIOD_MS;	//获取系统当前时间

    if (RosFrame_GetCmd(&cmd))
    {
        g.ros_cmd.vx_mps = cmd.vx_mps;
        g.ros_cmd.vy_mps = cmd.vy_mps;
        g.ros_cmd.wz_radps = cmd.wz_radps;
        g.ros_cmd.last_update_ms = now;
        g.ros_cmd.online = 1;

        g.ros_online = 1;
        s_last_rx_ms = now;
    }

	//超时清零
    if ((now - s_last_rx_ms) > ROS_TIMEOUT_MS)
    {
        g.ros_online = 0;
        g.ros_cmd.online = 0;
        g.ros_cmd.vx_mps = 0.0f;
        g.ros_cmd.vy_mps = 0.0f;
        g.ros_cmd.wz_radps = 0.0f;
    }

	//每10ms上报
    if ((now - s_last_tx_ms) >= ROS_UPLOAD_PERIOD_MS)
    {
        uint8_t tx_buf[ROS_TX_LEN];
        RosStatus_t status;

        //当前底盘速度
		status.vx_mps = g.feedback_speed.vx_mps;
        status.vy_mps = g.feedback_speed.vy_mps;
        status.wz_radps = g.feedback_speed.wz_radps;

        status.status = RosFrame_TakeErrorFlags();

        if (!g.ros_online || !g.ros_cmd.online)
            status.status |= ROS_STATUS_TIMEOUT;

        RosFrame_BuildStatus(tx_buf, &status);					//速度上报组帧
        UART3_SendArr(tx_buf, ROS_TX_LEN);

        s_last_tx_ms = now;
    }
	
	//每100ms上报
	if ((now - s_last_bat_tx_ms) >= ROS_BAT_UPLOAD_PERIOD_MS)
	{
		uint8_t tx_buf[ROS_BAT_TX_LEN];
		RosBattery_t bat;

		bat.voltage_mv = g.battery.voltage_mv;
		bat.current_ma = g.battery.current_ma;
		bat.percent = g.battery.percent;
		bat.flags = g.battery.flags;
		bat.status = 0;

		if ((g.battery.flags & 0x0001) != 0)
		{
			bat.status |= 0x01;
		}

		RosFrame_BuildBattery(tx_buf, &bat);					//电池上报组帧
		UART3_SendArr(tx_buf, ROS_BAT_TX_LEN);

		s_last_bat_tx_ms = now;
	}
}
