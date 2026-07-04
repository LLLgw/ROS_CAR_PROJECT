#ifndef SERIAL_H
#define SERIAL_H


#include <string.h>
#include <cstring>
#include <unistd.h> 
#include <fcntl.h> 
#include <termios.h>
#include <iostream>

using namespace std;


class  Serial
{
    private:
        int fd;
        struct termios newtio;
        unsigned int ch;

        static speed_t baud_flag(int baud_rate){
            switch(baud_rate) {
                case 9600:   return B9600;
                case 19200:  return B19200;
                case 38400:  return B38400;
                case 57600:  return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
                case 460800: return B460800;
                default: throw std::runtime_error("unsupported baud rate");
            }
        }

    public:

        Serial(string dev, int baud_rate){

            fd = open(dev.c_str(),O_RDWR|O_NOCTTY);
            if(fd < 0)
            {
                cout<<"seral_open_error"<<endl;
            }

            if(tcgetattr(fd,&newtio) != 0){
                close(fd);
                fd = -1;
                throw std::runtime_error("tcgetattr faild");
            }
            

            cfmakeraw(&newtio);

            newtio.c_cflag |= CLOCAL | CREAD;
            newtio.c_cflag &= ~CSIZE;
            newtio.c_cflag |= CS8;
            newtio.c_cflag &= ~PARENB;
            newtio.c_cflag &= ~CSTOPB;
            newtio.c_cflag &= ~CRTSCTS;
            
            newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
            newtio.c_cc[VTIME] = 1;
            newtio.c_cc[VMIN] = 1;

            speed_t speed = baud_flag(baud_rate);
            cfsetispeed(&newtio,speed);
            cfsetospeed(&newtio,speed);

            tcflush(fd, TCIFLUSH);

            if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
                perror("com set error");
            }

            cout<<"seral_open_succed"<<endl;
        }

        ~Serial(){
            if(fd > 0)
            {
                close(fd);
                cout << "Serial Port Closed!" << endl;
            }
        }

        bool serial_read_byte(uint8_t &byte)
        {
            fd_set rd;
            FD_ZERO(&rd);
            FD_SET(fd, &rd);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 50000;  // 50ms

            int ret = select(fd + 1, &rd, NULL, NULL, &timeout);
            if (ret < 0) {
                perror("select");
                return false;
            }

            if (ret == 0) {
                return false;
            }

            ssize_t n = read(fd, &byte, 1);
            if (n < 0) {
                perror("read");
                return false;
            }

            if (n == 0) {
                std::cerr << "read returned 0" << std::endl;
                return false;
            }

            return true;
        }

        int serial_stream_read(uint8_t * ptr,int len)
        {
            int result = 0;

            result = read(fd,ptr,len);
            if(result == -1){
                cout<<"seral_read_error"<<endl;
                return false;
            }

            return result;

        }

        void serial_send(unsigned char *p_send_buff,const int count)
        {
            int result = 0;
            result = write(fd,p_send_buff,count);

            if(result == -1){
                cout<<"seral_sendd_error"<<endl;
            }
        }


};
#endif