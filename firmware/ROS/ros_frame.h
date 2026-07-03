#ifndef __ROS_FRAME_H
#define __ROS_FRAME_H

#include <stdint.h>

#define ROS_FRAME_HEAD 0xAB             //帧头
#define ROS_FRAME_TAIL 0xAC             //帧尾
#define ROS_RX_LEN     17               //速度接收帧长度
#define ROS_TX_LEN     17               //速度发送帧长度
#define ROS_BAT_TX_LEN  13              //电池发送帧长度

#define ROS_STATUS_TIMEOUT     0x01     //速度帧接受超时标志
#define ROS_STATUS_BCC_ERROR   0x02     //BCC校验错误标志
#define ROS_STATUS_FRAME_ERROR 0x04     //帧完整错误标志

//速度接收结构体
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} RosCmd_t;

//速度上报结构体
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
    uint8_t status;
} RosStatus_t;

//电池上报结构体
typedef struct
{
    uint16_t voltage_mv;
    int16_t current_ma;
    uint8_t percent;
    uint16_t flags;
    uint8_t status;
} RosBattery_t;

void RosFrame_Init(void);
void RosFrame_InputByte(uint8_t byte);
uint8_t RosFrame_GetCmd(RosCmd_t *cmd);
uint8_t RosFrame_TakeErrorFlags(void);
void RosFrame_BuildStatus(uint8_t buf[ROS_TX_LEN], const RosStatus_t *status);
void RosFrame_BuildBattery(uint8_t buf[ROS_BAT_TX_LEN], const RosBattery_t *bat);

#endif
