#ifndef _ROBOT_INFO_TRANSFORM
#define _ROBOT_INFO_TRANSFORM

#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/string.hpp"

#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "robot_low_connect/serial.hpp"

#include <functional>


/*设备文件*/
#define DEV     "/dev/robot_mcu"
#define BAUD    115200

/*基础帧*/
#define BASE_FRAME_HEAD 0XAB   /*帧头*/
#define BASE_FRAME_TAIL 0XAC   /*帧尾*/
#define BASE_FRAME_SIZE 17 

/*发布帧*/
#define TX_FRAME_SIZE   17

/*基础帧数据结构*/
typedef struct {
    uint8_t data[BASE_FRAME_SIZE];

}Frame_base;

/*元素*/
typedef union _data
{
    float  fl_data;
    uint8_t u8_data[4];
}un_data;

/*stm32_imu*/
typedef struct{
    float Robot_ax;
    float Robot_ay;
    float Robot_az;
    float Robot_gx;
    float Robot_gy;
    float Robot_gz;
}imu;

/*encoder*/
typedef struct{
    un_data Robot_x;
    un_data Robot_y;
    un_data Robot_z;
}Robot;

/*协方差矩阵*/
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0,  1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3};

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0,  1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0,  1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3};
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0,  1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9};


class Info_tra : public rclcpp::Node
{
    public:
        Info_tra(std::string node_name);
        ~Info_tra() override = default;
        void  large_controll();
        Serial stm32_serial;
    
    private:
        bool       from_bottom_data();/*从地板读数据*/
        uint8_t    BCC_check(const uint8_t * ptr,int len);/*校验*/
        float      Frame_transfer(uint8_t high,uint8_t low);/*数据转换*/
        void       Pub_odom();/*发布odom*/

        void       Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist__);



        Frame_base                      Rx_Frame_base;/*接收缓冲*/
        imu                             Rx_imu;         
        Robot                           Rx_robot;     /*接收线速度x,y角速度z*/
        Robot                           Robot_pose;   /*位置x,y,z*/
        Robot                           Tx_robot;     /*发送线速度x,y角速度z*/
        Frame_base                      Tx_Frame;     /*发送缓冲*/
        rclcpp::TimerBase::SharedPtr    serial_timer_;
        rclcpp::Time                    Now, Last_Time;
        float                           Interval_time;
        std::string                     odom_frame_id,robot_frame_id;
        std::string                     odom_pub_topic,sub_vel_topic;
        std::thread                     read_thread_;
        std::atomic<bool>               running_;
        float                           scale;      /*odom比例系数*/
                          

        /*发布*/
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

        /*订阅*/
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_Sub;
};


#endif
