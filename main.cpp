#include <iostream>
#include <iostream>
#include "posital_encoder_modbus.hpp"
#include <memory>
int main(void)
{
    std::cout << "Endl" << std::endl;
    int baudrate = 19200;
    std::string parity = "E";
    int data_bit = 8;
    int stop_bit = 1;
    int debug = 1;
    std::string device_path = "/dev/ttyUSB0";
    int slave_id = 127;
    auto posital_encoder_modbus_ = std::make_unique<posital_encoder_modbus::PositalEncoderModbus>(
        device_path, baudrate, parity[0], data_bit, stop_bit, debug);
    posital_encoder_modbus_->init_encoder_motor(slave_id, 2, 1);
    while (true)
    {
        posital_encoder_modbus_->read_data();
        std::cout << "######################################" << std::endl;
        std::cout << "Pose: " << posital_encoder_modbus_->get_pose() << std::endl;
        std::cout << "Pose fail status: " << posital_encoder_modbus_->get_pose_fail_status() << std::endl;
        std::cout << "Speed: " << posital_encoder_modbus_->get_speed() << std::endl;
        std::cout << "Speed fail status: " << posital_encoder_modbus_->get_speed_fail_status() << std::endl;
        std::cout << "######################################\n"
                  << std::endl;
        sleep(0.5);
    }

    return -1;
}