#include "FreeRTOS.h"
#include "task.h"

#include "ros_frame.h"

static uint8_t s_rx_buf[ROS_RX_LEN];					//接收缓冲区
static uint8_t s_rx_index = 0;							//下标
static volatile uint8_t s_cmd_ready = 0;				//完整帧判断
static RosCmd_t s_last_cmd;								//解析帧速度指令

static volatile uint8_t s_bcc_error_flag = 0;			//校验判断
static volatile uint8_t s_frame_error_flag = 0;			//帧尾判断

//联合体，方便串口传输float，避免*1000压缩
typedef union
{
    float value;
    uint8_t bytes[4];
} Bytes_t;

//bcc校验
static uint8_t xor_check(const uint8_t *buf, uint8_t start, uint8_t end)
{
    uint8_t v = 0;
    uint8_t i;

    for (i = start; i <= end; i++)
        v ^= buf[i];

    return v;
}

////大端读取
//static int16_t get_i16_be(const uint8_t *buf, uint8_t index)
//{
//    return (int16_t)(((uint16_t)buf[index] << 8) | buf[index + 1]);
//}

//大端存储
static void put_i16_be(uint8_t *buf, uint8_t index, int16_t value)
{
    uint16_t raw;

    raw = (uint16_t)value;
    buf[index] = (uint8_t)(raw >> 8);
    buf[index + 1] = (uint8_t)(raw & 0xFF);
}

////浮点型转整型方便串口传输
//static int16_t float_to_i16_1000(float value)
//{
//    float scaled;
//    int32_t temp;

//    scaled = value * 1000.0f;

//    //四舍五入
//	if (scaled >= 0.0f)
//        temp = (int32_t)(scaled + 0.5f);
//    else
//        temp = (int32_t)(scaled - 0.5f);

//    //限幅
//	if (temp > 32767)
//        temp = 32767;
//    else if (temp < -32768)
//        temp = -32768;

//    return (int16_t)temp;
//}

//速度帧小端读取
static float get_bytes(const uint8_t *buf, uint8_t index)
{
    Bytes_t data;

    data.bytes[0] = buf[index + 0];
    data.bytes[1] = buf[index + 1];
    data.bytes[2] = buf[index + 2];
    data.bytes[3] = buf[index + 3];

    return data.value;
}

//速度帧小端存储
static void put_bytes(uint8_t *buf, uint8_t index, float value)
{
    Bytes_t data;

    data.value = value;

    buf[index + 0] = data.bytes[0];
    buf[index + 1] = data.bytes[1];
    buf[index + 2] = data.bytes[2];
    buf[index + 3] = data.bytes[3];
}

//判断标志初始化
void RosFrame_Init(void)
{
    s_rx_index = 0;
    s_cmd_ready = 0;
    s_bcc_error_flag = 0;
    s_frame_error_flag = 0;
}

//接收字节帧
void RosFrame_InputByte(uint8_t byte)
{
	//帧头判断
    if (s_rx_index == 0)
    {
        if (byte != ROS_FRAME_HEAD)
            return;
    }

    s_rx_buf[s_rx_index] = byte;
    s_rx_index++;

	if (s_rx_index < ROS_RX_LEN)
        return;

    s_rx_index = 0;

    //帧尾判断
	if (s_rx_buf[16] != ROS_FRAME_TAIL)
    {
        s_frame_error_flag = 1;
        return;
    }

	//bcc校验
    if (xor_check(s_rx_buf, 1, 14) != s_rx_buf[15])
    {
        s_bcc_error_flag = 1;
        return;
    }

    //底盘速度帧读取
	s_last_cmd.vx_mps = get_bytes(s_rx_buf, 3);
    s_last_cmd.vy_mps = get_bytes(s_rx_buf, 7);
    s_last_cmd.wz_radps = -get_bytes(s_rx_buf, 11);

    s_cmd_ready = 1;
}

//取指令
uint8_t RosFrame_GetCmd(RosCmd_t *cmd)
{
    if (cmd == 0)
        return 0;

    taskENTER_CRITICAL();				//关中断，取指令时候暂停串口接收防止s_last_cmd改变

    if (!s_cmd_ready)
    {
        taskEXIT_CRITICAL();
        return 0;
    }

    *cmd = s_last_cmd;
    s_cmd_ready = 0;

    taskEXIT_CRITICAL();				//开中断

    return 1;
}

//取错误标志
uint8_t RosFrame_TakeErrorFlags(void)
{
    uint8_t flags = 0;

    taskENTER_CRITICAL();

    if (s_bcc_error_flag)
    {
        flags |= ROS_STATUS_BCC_ERROR;
        s_bcc_error_flag = 0;
    }

    if (s_frame_error_flag)
    {
        flags |= ROS_STATUS_FRAME_ERROR;
        s_frame_error_flag = 0;
    }

    taskEXIT_CRITICAL();

    return flags;
}

//组速度上报帧
void RosFrame_BuildStatus(uint8_t buf[ROS_TX_LEN], const RosStatus_t *status)
{
    buf[0] = ROS_FRAME_HEAD;
    buf[1] = 0x01;
    buf[2] = status->status;

    put_bytes(buf, 3, status->vx_mps);
    put_bytes(buf, 7, status->vy_mps);
    put_bytes(buf, 11, status->wz_radps);

    buf[15] = xor_check(buf, 1, 14);
    buf[16] = ROS_FRAME_TAIL;
}

//组电池上报帧
void RosFrame_BuildBattery(uint8_t buf[ROS_BAT_TX_LEN], const RosBattery_t *bat)
{
    buf[0] = ROS_FRAME_HEAD;
    buf[1] = 0x02;
    buf[2] = bat->status;

    put_i16_be(buf, 3, (int16_t)bat->voltage_mv);
    put_i16_be(buf, 5, bat->current_ma);

    buf[7] = bat->percent;

    put_i16_be(buf, 8, (int16_t)bat->flags);

    buf[10] = 0;
    buf[11] = xor_check(buf, 1, 10);
    buf[12] = ROS_FRAME_TAIL;
}
