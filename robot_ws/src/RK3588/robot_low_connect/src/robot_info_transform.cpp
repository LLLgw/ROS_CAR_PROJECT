#include"robot_low_connect/robot_info_transform.hpp"
#include<iostream>

/*BCC校验*/ 
 uint8_t Info_tra::BCC_check(const uint8_t *ptr,int len){
    int num = len - 3;
    uint8_t check_= 0;
    for(int i=1; i <= num; i++)
    {
        check_ = ptr[i] ^ check_;
    }

    return check_;

}

/*数据转换*/
/*
float  Info_tra::Frame_transfer(uint8_t high,uint8_t low){
    short temp = 0;
    float data=0;

    temp = high<<8;
    temp |= low;
    data = temp/1000 + temp%1000*0.001;

    return data;
}*/


/*更新数据*/
bool Info_tra::from_bottom_data(){

    uint8_t  head_ptr;/*读一个字节*/

    stm32_serial.serial_read_byte(head_ptr);

    //RCLCPP_INFO(this->get_logger(),"data: %x ",head_ptr);
    static int count_base=0;

    /*基础帧开始*/
    if(count_base == 0 && head_ptr == BASE_FRAME_HEAD)
    {
        Rx_Frame_base.data[count_base++] = head_ptr;
    }
    else if(count_base > 0){
        Rx_Frame_base.data[count_base++] = head_ptr;
    }

    /*收到完整帧*/
    if(count_base == BASE_FRAME_SIZE){
        if(Rx_Frame_base.data[BASE_FRAME_SIZE -1] == BASE_FRAME_TAIL)
        {
            if(BCC_check(Rx_Frame_base.data,BASE_FRAME_SIZE) == Rx_Frame_base.data[BASE_FRAME_SIZE - 2])
            {
                
                /*线速度和角速度*/
                /*
                Rx_robot.Robot_x = Frame_transfer(Rx_Frame_base.data[3],Rx_Frame_base.data[4]);
                Rx_robot.Robot_y = Frame_transfer(Rx_Frame_base.data[5],Rx_Frame_base.data[6]);
                Rx_robot.Robot_z = Frame_transfer(Rx_Frame_base.data[7],Rx_Frame_base.data[8]);*/

                /*使用驱动板MPU6050*/
                /*
                Rx_imu.Robot_ax = Frame_transfer(Rx_Frame_base.data[9],Rx_Frame_base.data[10]);
                Rx_imu.Robot_ay = Frame_transfer(Rx_Frame_base.data[11],Rx_Frame_base.data[12]);
                Rx_imu.Robot_az = Frame_transfer(Rx_Frame_base.data[13],Rx_Frame_base.data[14]);
                Rx_imu.Robot_gx = Frame_transfer(Rx_Frame_base.data[15],Rx_Frame_base.data[16]);
                Rx_imu.Robot_gy = Frame_transfer(Rx_Frame_base.data[17],Rx_Frame_base.data[18]);
                Rx_imu.Robot_gz = Frame_transfer(Rx_Frame_base.data[19],Rx_Frame_base.data[20]);*/

                /*线速度和角速度*/
                Rx_robot.Robot_x.u8_data[0] = Rx_Frame_base.data[3];
                Rx_robot.Robot_x.u8_data[1] = Rx_Frame_base.data[4];
                Rx_robot.Robot_x.u8_data[2] = Rx_Frame_base.data[5];
                Rx_robot.Robot_x.u8_data[3] = Rx_Frame_base.data[6];

                Rx_robot.Robot_y.u8_data[0] = Rx_Frame_base.data[7];
                Rx_robot.Robot_y.u8_data[1] = Rx_Frame_base.data[8];
                Rx_robot.Robot_y.u8_data[2] = Rx_Frame_base.data[9];
                Rx_robot.Robot_y.u8_data[3] = Rx_Frame_base.data[10];

                Rx_robot.Robot_z.u8_data[0] = Rx_Frame_base.data[11];
                Rx_robot.Robot_z.u8_data[1] = Rx_Frame_base.data[12];
                Rx_robot.Robot_z.u8_data[2] = Rx_Frame_base.data[13];
                Rx_robot.Robot_z.u8_data[3] = Rx_Frame_base.data[14];


                count_base =0;
                memset(Rx_Frame_base.data,0,sizeof(Rx_Frame_base.data));
                return true;
            }
        }

        memset(Rx_Frame_base.data,0,sizeof(Rx_Frame_base.data));
        count_base =0;
    }


    return false;
}

/*发布odom*/
void Info_tra::Pub_odom(){
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_pose.Robot_z.fl_data);/*转换为四元数*/
    geometry_msgs::msg::Quaternion  odom_quat = tf2::toMsg(q);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;
    odom.pose.pose.position.x = Robot_pose.Robot_x.fl_data;
    odom.pose.pose.position.y = Robot_pose.Robot_y.fl_data;
    odom.pose.pose.position.z = Robot_pose.Robot_z.fl_data;
    odom.pose.pose.orientation = odom_quat;/*四元数*/

    odom.child_frame_id = robot_frame_id;/*子坐标*/
    odom.twist.twist.linear.x = Rx_robot.Robot_x.fl_data;
    odom.twist.twist.linear.y = Rx_robot.Robot_y.fl_data;
    odom.twist.twist.angular.z = Rx_robot.Robot_z.fl_data;

    /*静态下使用编码器数据,运动时使用imu数据*/
    if(Rx_robot.Robot_x.fl_data == 0 && Rx_robot.Robot_y.fl_data == 0 && Rx_robot.Robot_z.fl_data == 0)
    {
        memcpy(&odom.pose.covariance,odom_pose_covariance2,sizeof(odom_pose_covariance2));
        memcpy(&odom.twist.covariance,odom_twist_covariance2,sizeof(odom_twist_covariance2));
    }else{
        memcpy(&odom.pose.covariance,odom_pose_covariance,sizeof(odom_pose_covariance));
        memcpy(&odom.twist.covariance,odom_twist_covariance,sizeof(odom_twist_covariance));
    }

    odom_publisher->publish(odom);

}


/*循环控制*/
void Info_tra::large_controll(){


    while( rclcpp::ok() && running_ ){

        if(from_bottom_data() == true)
        {
            //RCLCPP_INFO(this->get_logger(),"Frame received finished");

            Now =  rclcpp::Node::now();
            Interval_time = (Now - Last_Time).seconds();
            
            /*停住时里程计会摆动*/
            if(abs(Rx_robot.Robot_x.fl_data) < 0.01){
                Rx_robot.Robot_x.fl_data = 0;
            }

            /*里程计误差*/
            Rx_robot.Robot_x.fl_data  =  Rx_robot.Robot_x.fl_data* scale;

            /*计算odom坐标*/
            Robot_pose.Robot_x.fl_data += Rx_robot.Robot_x.fl_data * cos(Rx_robot.Robot_z.fl_data) * Interval_time;
            Robot_pose.Robot_y.fl_data += Rx_robot.Robot_x.fl_data * sin(Rx_robot.Robot_z.fl_data) * Interval_time;
            Robot_pose.Robot_z.fl_data += Rx_robot.Robot_z.fl_data * Interval_time;

            Pub_odom();

            Last_Time = Now;

        }
    }

}


/*订阅cmd_vel话题*/
void Info_tra::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist__){

    /*帧头帧尾*/
    Tx_Frame.data[0] = BASE_FRAME_HEAD;
    Tx_Frame.data[TX_FRAME_SIZE-1] = BASE_FRAME_TAIL;

    /*保留*/
    Tx_Frame.data[1] = 0;
    Tx_Frame.data[2] = 0;

    Tx_robot.Robot_x.fl_data = twist__->linear.x;
    Tx_robot.Robot_y.fl_data = twist__->linear.y;
    Tx_robot.Robot_z.fl_data = twist__->angular.z;

    Tx_Frame.data[3] = Tx_robot.Robot_x.u8_data[0];
    Tx_Frame.data[4] = Tx_robot.Robot_x.u8_data[1];
    Tx_Frame.data[5] = Tx_robot.Robot_x.u8_data[2];
    Tx_Frame.data[6] = Tx_robot.Robot_x.u8_data[3];

    Tx_Frame.data[7] = Tx_robot.Robot_y.u8_data[0];
    Tx_Frame.data[8] = Tx_robot.Robot_y.u8_data[1];
    Tx_Frame.data[9] = Tx_robot.Robot_y.u8_data[2];
    Tx_Frame.data[10] = Tx_robot.Robot_y.u8_data[3];

    Tx_Frame.data[11] = Tx_robot.Robot_z.u8_data[0];
    Tx_Frame.data[12] = Tx_robot.Robot_z.u8_data[1];
    Tx_Frame.data[13] = Tx_robot.Robot_z.u8_data[2];
    Tx_Frame.data[14] = Tx_robot.Robot_z.u8_data[3];


    Tx_Frame.data[15] = BCC_check(Tx_Frame.data,TX_FRAME_SIZE);

    try{
         stm32_serial.serial_send(Tx_Frame.data,TX_FRAME_SIZE);
         /*RCLCPP_INFO(this->get_logger(),"send cmd_vel ...");*/
    }catch(...){
    }
   
    memset(&Tx_Frame,0,sizeof(Tx_Frame));
}



using namespace std::chrono_literals;
using std::placeholders::_1;
/*构造函数*/

Info_tra::Info_tra(std::string node_name) : rclcpp::Node(node_name),stm32_serial(DEV,BAUD){
    /*循环控制定时器*/
    //serial_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&Info_tra::large_controll, this));

    Interval_time = 0;

    memset(&Rx_Frame_base,0,sizeof(Rx_Frame_base));
    memset(&Rx_imu,0,sizeof(Rx_imu));
    memset(&Rx_robot,0,sizeof(Rx_robot));
    memset(&Robot_pose,0,sizeof(Robot_pose));
    memset(&Tx_Frame,0,sizeof(Tx_Frame));
    memset(&Tx_robot,0,sizeof(Tx_robot));

    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
    this->declare_parameter<std::string>("odom_pub_topic", "/odom/raw");
    this->declare_parameter<std::string>("sub_vel_topic", "/cmd_vel");

    this->get_parameter("odom_frame_id", odom_frame_id);
    this->get_parameter("robot_frame_id", robot_frame_id);
    this->get_parameter("odom_pub_topic", odom_pub_topic);
    this->get_parameter("sub_vel_topic", sub_vel_topic);

    /*里程计误差的比列系数*/
    scale= 1.129;


    odom_publisher = create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic , 2);/*创建odom话题发布*/

    vel_Sub = create_subscription<geometry_msgs::msg::Twist>(sub_vel_topic,5,std::bind(&Info_tra::Cmd_Vel_Callback,this, _1));/*订阅速度*/

    running_ = true;
    read_thread_ = std::thread(&Info_tra::large_controll,this);

    Last_Time = rclcpp::Node::now();

    RCLCPP_INFO(this->get_logger(), "Info_tra 节点及串口初始化成功！");
}



int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto  robot = std::make_shared<Info_tra>("robot_info_transform");
    
    rclcpp::spin(robot);
    rclcpp::shutdown();
    return 0;
}