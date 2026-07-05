#ifndef __STATE_H
#define __STATE_H

#include "stm32f4xx.h"
#include <stdint.h>

//指令控制源
typedef enum
{
    CTRL_SOURCE_NONE=0,                    //无控制源
    CTRL_SOURCE_ROS,                       //ROS控制
    CTRL_SOURCE_PS2,                       //PS2手柄控制
} ControlSource_t;

//运动模式
typedef enum
{
    MOTION_MODE_STOP=0,                    //停止
    MOTION_MODE_ROS,                       //ROS模式
    MOTION_MODE_PS2,                       //PS2模式
} MotionMode_t;

//底盘指令
typedef struct
{
    float vx_mps;                          //x轴线速度，m/s
    float vy_mps;                          //y轴线速度，m/s
    float wz_radps;                        //z轴角速度，rad/s
    uint32_t last_update_ms;               //上一次更新时间戳
    uint8_t online;                        //在线状态
} ChassisCmd_t;

//底盘反馈速度，用于显示与上报闭环
typedef struct
{
    float vx_mps;                          //x轴线速度，m/s
    float vy_mps;                          //y轴线速度，m/s
    float wz_radps;                        //z轴角速度，rad/s
} ChassisSpeed_t;

//电池状态
typedef struct
{
    uint16_t voltage_mv;                   //电压，mv
    int16_t current_ma;                    //电流，ma，正号放电负号充电
    uint8_t percent;                       //电量百分比
    uint16_t flags;                        //状态标志
} BatteryState_t;

//全局状态
typedef struct
{
    //控制指令
    ChassisCmd_t ros_cmd;                  //ros指令
    ChassisCmd_t ps2_cmd;                  //手柄指令
    ChassisCmd_t final_cmd;                //最终指令

    //反馈
    ChassisSpeed_t feedback_speed;         //底盘实际速度反馈
    BatteryState_t battery;                //电池状态反馈

    //控制状态
    ControlSource_t control_source;        //当前控制源
    MotionMode_t motion_mode;              //当前运动模式
    ControlSource_t selected_source;       //用户选择的控制源

    //在线状态
    uint8_t ros_online;                    //ros通讯在线状态
    uint8_t ps2_online;                    //手柄在线状态

    //保护标志
    uint8_t motor_enable;                  //电机转动使能
    uint8_t emergency_stop;                //软件急停标志
    uint8_t low_battery;                   //低电标志
} State_t;

extern State_t g;                          //全局变量

void State_Init(void);

#endif
