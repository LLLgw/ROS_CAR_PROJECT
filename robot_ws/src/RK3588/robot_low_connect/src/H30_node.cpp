#include "robot_low_connect/H30_node.hpp"

/*校验和*/
bool  H30_::check_sum(const uint8_t *ptr,int len){
    uint8_t  CK_1 = 0,CK_2 = 0;

    for(int i=0; i < len;i++){
        CK_1 +=  ptr[i];
        CK_2 +=  CK_1;
    }

    if(CK_1 == Rx_data[SIZE-2] && CK_2 == Rx_data[SIZE-1]) return true;

    return false;
}


/*数据转换,高位为fourth*/
float  H30_::transfer_data(uint8_t fourth_byte,uint8_t third_byte,uint8_t second_byte,uint8_t first_byte){
    int  temp=0;
    float result = 0;
    temp = fourth_byte;
    temp = (temp << 8) | third_byte;
    temp = (temp << 8) | second_byte;
    temp = (temp << 8) | first_byte;

    result = temp * 0.000001;

    return result;
}

/*数据处理*/
void H30_::imu_process(){
     int location = 0;
     static int count = 0;
    
    while(location < rx_size){
        //RCLCPP_INFO(this->get_logger(),"找帧头");
        /*找帧头*/
        if(count == 0 && Rx_cache[location] == FRAME_H30_HEAD_1){
            Rx_data[count++] = Rx_cache[location++];
            continue;
        }

        if(count == 1 ){
            if(Rx_cache[location] == FRAME_H30_HEAD_2){
                Rx_data[count++] = Rx_cache[location++];
            }
            else{
                count = 0;
                location++;
                memset(Rx_data,0,sizeof(Rx_data));
                continue;
            }
        }

        /*已找到帧头*/
        Rx_data[count++] = Rx_cache[location++];

        /*已读完整帧*/
        if(count == SIZE){
            if(check_sum(Rx_data+2,SIZE-4) == true){
                //valid_data_len = Rx_data[4];
                if(Rx_data[5] == ACCELERATION){
                    imu_data.velocity_a.x = transfer_data(Rx_data[10],Rx_data[9],Rx_data[8],Rx_data[7]);
                    imu_data.velocity_a.y = transfer_data(Rx_data[14],Rx_data[13],Rx_data[12],Rx_data[11]);
                    imu_data.velocity_a.z = transfer_data(Rx_data[18],Rx_data[17],Rx_data[16],Rx_data[15]);
                }
                if(Rx_data[19] == ANGULAR_V){
                    imu_data.velocity_g.x = transfer_data(Rx_data[24],Rx_data[23],Rx_data[22],Rx_data[21]);
                    imu_data.velocity_g.y = transfer_data(Rx_data[28],Rx_data[27],Rx_data[26],Rx_data[25]);
                    imu_data.velocity_g.z = transfer_data(Rx_data[32],Rx_data[31],Rx_data[30],Rx_data[29]);
                }
                if(Rx_data[33] == QUATERNION){
                    imu_data.q.q0 = transfer_data(Rx_data[38],Rx_data[37],Rx_data[36],Rx_data[35]);
                    imu_data.q.q1 = transfer_data(Rx_data[42],Rx_data[41],Rx_data[40],Rx_data[39]);
                    imu_data.q.q2 = transfer_data(Rx_data[46],Rx_data[45],Rx_data[44],Rx_data[43]);
                    imu_data.q.q3 = transfer_data(Rx_data[50],Rx_data[49],Rx_data[48],Rx_data[47]);
                }

                /*发布帧*/
                pub_imu();

            }
            count = 0;
            memset(Rx_data,0,sizeof(Rx_data));
        }
        
    }

    memset(Rx_cache,0,sizeof(Rx_cache));
}


/*imu发布*/
void H30_::pub_imu(){
    
    //RCLCPP_INFO(this->get_logger(),"发布imu");
    
    /*
    RCLCPP_INFO(this->get_logger(),"ax:%f",imu_data.velocity_a.x);
    RCLCPP_INFO(this->get_logger(),"ay:%f",imu_data.velocity_a.y);
    RCLCPP_INFO(this->get_logger(),"az:%f",imu_data.velocity_a.z);
    RCLCPP_INFO(this->get_logger(),"ax:%f",imu_data.velocity_g.x);
    RCLCPP_INFO(this->get_logger(),"ay:%f",imu_data.velocity_g.y);
    RCLCPP_INFO(this->get_logger(),"az:%f",imu_data.velocity_g.z);
    RCLCPP_INFO(this->get_logger(),"q0:%f",imu_data.q.q0);
    RCLCPP_INFO(this->get_logger(),"q1:%f",imu_data.q.q1);
    RCLCPP_INFO(this->get_logger(),"q2:%f",imu_data.q.q2);
    RCLCPP_INFO(this->get_logger(),"q3:%f",imu_data.q.q3);
    */

    sensor_msgs::msg::Imu Imu_Data_pub;
    Imu_Data_pub.header.stamp = rclcpp::Node::now();
    Imu_Data_pub.header.frame_id = gyro_frame_id;

    /*四元数*/
    Imu_Data_pub.orientation.x = imu_data.q.q1;
    Imu_Data_pub.orientation.y = imu_data.q.q2;
    Imu_Data_pub.orientation.z = imu_data.q.q3;
    Imu_Data_pub.orientation.w = imu_data.q.q0;

    /*协方差*/
    Imu_Data_pub.orientation_covariance[0] = 1e6;
    Imu_Data_pub.orientation_covariance[4] = 1e6;
    Imu_Data_pub.orientation_covariance[8] = 1e-6;

    /*角速度*/
    Imu_Data_pub.angular_velocity.x = imu_data.velocity_g.x / 57.3;
    Imu_Data_pub.angular_velocity.y = imu_data.velocity_g.y / 57.3;
    Imu_Data_pub.angular_velocity.z = imu_data.velocity_g.z / 57.3;
    
    /*角速度协方差*/
    Imu_Data_pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_pub.angular_velocity_covariance[8] = 1e-6;

    /*线性加速度*/
    Imu_Data_pub.linear_acceleration.x = imu_data.velocity_a.x;
    Imu_Data_pub.linear_acceleration.y = imu_data.velocity_a.y;
    Imu_Data_pub.linear_acceleration.z = imu_data.velocity_a.z;

    /*发布imu topic话题*/
    imu_publisher->publish(Imu_Data_pub);

    memset(&imu_data,0,sizeof(imu_data));
}

/*大循环控制*/
void H30_::read_loop(){
    while(rclcpp::ok() && running_ ){
        
        int result = 0;
        /*读数据*/
        result = H30_serial.serial_stream_read(Rx_cache,SIZE);
        //RCLCPP_INFO(this->get_logger(),"result : %d",result);
        if(result <= 0){
            memset(Rx_cache,0,SIZE);
            continue;
        }
       
        rx_size = result;
        /*数据解析*/
        imu_process();

    }
}

/*构造函数*/
H30_::H30_(std::string node_name) : rclcpp::Node(node_name),H30_serial(dev,BAUD){
    
    memset(Rx_data,0,sizeof(Rx_data));
    memset(Rx_cache,0,sizeof(Rx_cache));
    memset(&imu_data,0,sizeof(imu_data));

    gyro_frame_id = "imu_link";

    imu_publisher =  create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2);
    
    running_ = true;
    read_thread_ = std::thread(&H30_::read_loop,this);

    RCLCPP_INFO(this->get_logger(),"H30 serial Inited");
}


H30_::~H30_()
{
    running_ = false;

    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}


int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    auto H30 = std::make_shared<H30_>("H30_node");
    rclcpp::spin(H30);
    rclcpp::shutdown();
    return 0;
}