#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "string.h"
#include "robot_low_connect/serial.hpp"
#include <sensor_msgs/msg/imu.hpp>

#define BAUD    115200
#define dev     "/dev/h30_imu"
#define SIZE   53

/*帧头*/
#define FRAME_H30_HEAD_1    0X59 
#define FRAME_H30_HEAD_2    0X53

#define ACCELERATION        0X10 /*加速度*/
#define ANGULAR_V           0X20 /*角速度*/
#define QUATERNION          0X41 /*四元数*/

enum{cache_end,cache_running,frame_end};

typedef struct{
    float x;
    float y;
    float z;
}velocity;

typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;
}qua;/*四元数*/

typedef struct{
    qua  q;
    velocity velocity_a;
    velocity velocity_g;
}imu_;


class H30_ : public rclcpp::Node{
    public:
        H30_(std::string node_name);
        ~H30_() override;
        Serial  H30_serial;
    
    private:
        
        void    read_loop();    /*imu线程*/  
        void    imu_process();  /*处理帧*/
        bool    check_sum(const uint8_t *ptr, int len); /*校验*/
        float   transfer_data(uint8_t fourth_byte,uint8_t third_byte,uint8_t second_byte,uint8_t first_byte); /*类型转换*/
        void    pub_imu();  /*发布imu*/

        std::thread read_thread_;
        std::atomic<bool> running_{false};
        std::vector<uint8_t> rx_buffer_;
        int16_t rx_size;
        uint8_t Rx_cache[SIZE]; /*循环接收缓冲区*/
        uint8_t Rx_data[SIZE];  /*整帧区*/
        imu_    imu_data;
        string gyro_frame_id;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;

};