#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "string.h"
#include "robot_low_connect/serial.hpp"
#include "robot_low_connect/msg/gpgga.hpp"

#define BOND 9600
#define DEV "/dev/g60_gps"
 
typedef struct {
    float lat;  /*纬度*/
    char  ulat; /*北纬或南纬*/
    float lon;  /*精度*/
    char  ulon; /*东经或西经*/
    float curr_time; /*当前时间*/
}location_struct;


class G60_ : public rclcpp::Node{
    public:        
        G60_(std::string node_name);
        ~G60_() override;
        Serial G60_serial;

    private:
        void    read_loop();/*线程*/
        void    process();
        int16_t check();/*校验*/
        void    transform();
        float   char_to_in(int start,int end);
        void    pub_location();

        robot_low_connect::msg::Gpgga gga_msg; /*GGA数据*/
        std::thread read_thread_;
        std::atomic<bool> running_{false};
        std::vector<uint8_t> rx_buffer_;/*接收缓冲区*/
        location_struct cache_data;

        rclcpp::Publisher<robot_low_connect::msg::Gpgga>::SharedPtr GGA_Publisher;

};