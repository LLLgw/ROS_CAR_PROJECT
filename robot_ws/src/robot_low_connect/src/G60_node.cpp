#include"robot_low_connect/G60_node.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

int16_t G60_::check(){
    
    int i=0;
    uint8_t check_num=0;
    for(i=1;rx_buffer_[i] != '*';i++){
        check_num ^= rx_buffer_[i]; 
    }

    return check_num;
}

/*类型转换*/
float G60_::char_to_in(int start,int end){

    int   int_data = 0;
    float fl_data = 0,fl_scale=1;
    int   mod = 0;

    while(start < end){

        /*小数部分*/
        if(mod == 1){
             fl_data  = fl_data +(rx_buffer_[start]-'0')*fl_scale;
             fl_scale = fl_scale * 0.1;

             start++;
             continue;
        }
        
        if(rx_buffer_[start] == '.' ){
            mod = 1;
            start++;
            continue;
        }

        /*整数部分*/
        int_data = int_data +(rx_buffer_[start]-'0');
        int_data = int_data * 10;
        start++;
    }

    int_data= int_data / 10;

    return int_data+fl_data;
}

/*转换*/
void G60_::transform(){
    
    int i=0;
    int count_=0;
    int start = 0;

    while(rx_buffer_[i] != '*'){

        //RCLCPP_INFO(this->get_logger(),"%c",rx_buffer_[i]);
        
        if(rx_buffer_[i] == ',') {
            count_++;

            if(start +1 == i){
                start = i;
                i++;
                continue;
            } 

            /*处理时间*/
            if(count_ == 2){
               cache_data.curr_time = char_to_in(start+1 , i);
            }
            /*纬度*/
            else if(count_ == 3){
                cache_data.lat = char_to_in(start+1 , i);
            }

            else if(count_ == 4){
                cache_data.ulat = rx_buffer_[i];
            }

            /*经度*/
            else if(count_ == 5){
                cache_data.lon = char_to_in(start+1,i);
            }

            else if(count_ == 6){
                cache_data.ulon = rx_buffer_[i];
            }

            start = i;
        }

        i++;
    }
}

void G60_::process(){
    uint8_t read_ptr;
    static int  count = 0;
    uint8_t check_num=0;
    const static char com[] = "GGA";

    G60_serial.serial_read_byte(read_ptr);
    //RCLCPP_INFO(this->get_logger(),"%c",read_ptr);


    if((count == 0  && read_ptr == '$') || count > 0){
        rx_buffer_.push_back(read_ptr); 
        count++;
    }

    if(count == 6){
        if(com[0] != rx_buffer_[3] || com[1] != rx_buffer_[4] ||  com[2] !=rx_buffer_[5]){
            rx_buffer_.clear();
            count = 0;
            return;
        }
    }

    /*找到GGA*/
    if(count > 6 && read_ptr == '\r'){

        if('A' <= rx_buffer_[count-3] && rx_buffer_[count-3] <= 'F'){
            switch(rx_buffer_[count-3]){
                case 'A': check_num = 10;break;
                case 'B': check_num = 11;break;
                case 'C': check_num = 12;break;
                case 'D': check_num = 13;break;
                case 'E': check_num = 14;break;
                case 'F': check_num = 15;break;
            };
            check_num = check_num << 4;
        }else{
             check_num = (rx_buffer_[count-3]-'0') << 4;
        }

        if('A' <= rx_buffer_[count-2] && rx_buffer_[count-2] <= 'F'){
            switch(rx_buffer_[count-2]){
                case 'A': check_num |= 10;break;
                case 'B': check_num |= 11;break;
                case 'C': check_num |= 12;break;
                case 'D': check_num |= 13;break;
                case 'E': check_num |= 14;break;
                case 'F': check_num |= 15;break;
            };
        }else{
            check_num = check_num | (rx_buffer_[count-2]-'0');
        }
       
       
        //RCLCPP_INFO(this->get_logger(),"chec_num : %x",check_num);

        if(check() == check_num){

            transform();

            if(cache_data.lat == 0 || cache_data.lon == 0){
                pub_location();
            }
            
            count = 0;
            check_num = 0;
            rx_buffer_.clear();
        }
    }


}

/*发布获取的位置和时间*/
void G60_::pub_location(){

    float temp = 0;

    /*id*/
    gga_msg.message_id = "gps";

    /*纬度*/
    gga_msg.lat = cache_data.lat / 100;
    temp = cache_data.lat - gga_msg.lat*100;
    gga_msg.lat = gga_msg.lat + temp/60.0;
    if(cache_data.ulat == 'S')  gga_msg.lat = -gga_msg.lat;

    /*经度*/
    gga_msg.lon = cache_data.lon / 100;
    temp = cache_data.lon - gga_msg.lon*100;
    gga_msg.lon = gga_msg.lon + temp/60.0;
    if(cache_data.ulon == 'W') gga_msg.lon = -gga_msg.lon;

    /*时间*/
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&t);
   
    /*gps没有获取到时间*/
    if(cache_data.curr_time == 0){
        gga_msg.hour = local_tm.tm_hour;
        gga_msg.min = local_tm.tm_min;
        gga_msg.seconds = local_tm.tm_sec;
    }
    else{
        gga_msg.hour = cache_data.curr_time/10000;
        gga_msg.min = cache_data.curr_time/100 - gga_msg.hour*100;
        gga_msg.seconds = cache_data.curr_time - gga_msg.hour*10000 - gga_msg.min*100;
    }

    gga_msg.day = local_tm.tm_mday;

    GGA_Publisher->publish(gga_msg);
}

/*线程*/
void G60_::read_loop(){
    while(rclcpp::ok() && running_){
        //RCLCPP_INFO(this->get_logger(),"running");
         process();
    }
}


G60_:: G60_(string name):rclcpp::Node(name),G60_serial(DEV,BOND){

    memset(&cache_data,0,sizeof(cache_data));

    GGA_Publisher = create_publisher<robot_low_connect::msg::Gpgga>("location",2);

    running_ = true;
    read_thread_ = std::thread(&G60_::read_loop,this);

    RCLCPP_INFO(this->get_logger(),"G60 init finish");
} 

G60_:: ~G60_(){
    running_ = false;
}

int main(int argc , char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<G60_>("G60_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}