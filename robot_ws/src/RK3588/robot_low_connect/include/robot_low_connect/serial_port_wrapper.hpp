#ifndef SERIAL_PORT_WRAPPER_HPP_
#define SERIAL_PORT_WRAPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "io_context/io_context.hpp"
#include "serial_driver/serial_driver.hpp"

class SerialPortWrapper
{
    public:
        SerialPortWrapper(const std::string & port_name, int baud_rate) : io_context_(1),serial_driver_(io_context_)
        {
            drivers::serial_driver::SerialPortConfig config(
            static_cast<uint32_t>(baud_rate),
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE);

            serial_driver_.init_port(port_name, config);
            serial_driver_.port()->open();
        }

        ~SerialPortWrapper()
        {
            close();
            io_context_.waitForExit();
        }

        bool isOpen() const
        {
            return serial_driver_.port() && serial_driver_.port()->is_open();
        }

        void close()
        {
            if (serial_driver_.port() && serial_driver_.port()->is_open()) {
            serial_driver_.port()->close();
            }
        }

        bool sendBytes(const uint8_t * data, std::size_t len)
        {
            try {
                std::vector<uint8_t> buffer(data, data + len);
                serial_driver_.port()->send(buffer);
                return true;
            } catch (...) {
                return false;
            }
        }

        bool receiveByte(uint8_t & byte)
        {
            try {
                std::vector<uint8_t> buf(1);
                std::size_t n = serial_driver_.port()->receive(buf);
                if (n == 1) {
                    byte = buf[0];
                    return true;
                }
                return false;
            } catch (...) {
                return false;
            }
        }

    private:
        drivers::common::IoContext io_context_;
        drivers::serial_driver::SerialDriver serial_driver_;
};

#endif