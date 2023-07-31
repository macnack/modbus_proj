// Copyright 2023 Mobile Robots Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "posital_encoder_modbus.hpp"

#include <cmath>
#include <iostream>
#define GEARBOX_RATIO 0.75f

void printBinary(uint32_t num)
{
  for (int i = 31; i >= 0; i--) {
    uint32_t mask = 1u << i;
    putchar((num & mask) ? '1' : '0');
  }
}

uint32_t combine_LSB_MSB(uint16_t LSB, uint16_t MSB)
{
  // Combine the MSB and LSB values into a 32-bit integer
  uint32_t combined_value = (static_cast<uint32_t>(MSB) << 16) | LSB;
  return combined_value;
}
float_t to_degree(uint32_t value)
{
  return static_cast<float_t>(value) * 360.0f / 65536.0f;
}

namespace posital_encoder_modbus
{
PositalEncoderModbus::PositalEncoderModbus() : last_pose_(0.0f), last_speed_(0.0f)
{
  // Empty constructor
}

PositalEncoderModbus::PositalEncoderModbus(
  std::string device, int baud, char parity, int data_bit, int stop_bit)
: last_pose_(0.0f), last_speed_(0.0f)
{
  dongle_ = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
  if (dongle_ == NULL) {
    std::cout << "Unable to create the libmodbus context\n" << std::endl;
  }
  if (modbus_connect(dongle_) == -1) {
    std::cout << "connection failed after init" << modbus_strerror(errno) << std::endl;
    // modbus_free(dongle_);
  } else {
    std::cout << "Connection done" << std::endl;
  }
}

bool PositalEncoderModbus::init_encoder_motor(
  int slave_id, int speed_mode_value, int speed_filter_value)
{
  // connection failed
  if (modbus_connect(dongle_) == -1) {
    std::cout << "connection failed" << std::endl;
    // modbus_free(dongle_);
    return true;
  }
  // set slave id
  if (modbus_set_slave(dongle_, slave_id) == -1) {
    // error invalid slave id
    std::cout << "invalid slave id" << std::endl;
    return true;
  }
  // write speed mode
  modbus_write_register(dongle_, 26, speed_mode_value);
  // speed filter
  modbus_write_register(dongle_, 27, speed_filter_value);
  uint32_t old_response_to_sec;
  uint32_t old_response_to_usec;

  modbus_get_response_timeout(dongle_, &old_response_to_sec, &old_response_to_usec);
  std::cout << "old response timeout: " << old_response_to_sec << " " << old_response_to_usec
            << std::endl;

  return false;
}

void PositalEncoderModbus::read_data()
{
  if (modbus_connect(dongle_) != -1) {
    std::vector<uint16_t> tab_reg(2);
    // read pose registers
    int req = modbus_read_registers(dongle_, 1, 2, tab_reg.data());
    if (req == -1) {
      std::cout << "Pose read failed." << std::endl;
      std::cout << modbus_strerror(errno) << std::endl;
      pose_fail_status_ = true;
      pose_ = last_pose_;
      return;
    }
    if (req == 2) {
      pose_ = to_degree(combine_LSB_MSB(tab_reg[1], tab_reg[0]));
      pose_fail_status_ = false;
      last_pose_ = pose_;
    } else {
      std::cout << "it was corrupted" << std::endl;
      pose_fail_status_ = true;
      pose_ = last_pose_;
    }
    // read speed registers
    req = modbus_read_registers(dongle_, 5, 2, tab_reg.data());
    if (req == -1) {
      std::cout << "Speed read failed." << std::endl;
      std::cout << modbus_strerror(errno) << std::endl;
      speed_fail_status_ = true;
      speed_ = last_speed_;
      return;
    }

    if (req == 2) {
      speed_ = to_degree(combine_LSB_MSB(tab_reg[1], tab_reg[0]));
      speed_fail_status_ = false;
      last_speed_ = speed_;
    } else {
      std::cout << "it was corrupted" << std::endl;
      speed_fail_status_ = true;
      speed_ = last_speed_;
    }
  } else {
    pose_fail_status_ = true;
    pose_ = last_pose_;
    speed_fail_status_ = true;
    speed_ = last_speed_;
  }
}

void PositalEncoderModbus::read_data(
  float & speed, bool & speed_fail_status, float & pose, bool & pose_fail_status)
{
  if (modbus_connect(dongle_) != -1) {
    uint16_t tab_reg[32];
    // read pose registers
    int req = modbus_read_registers(dongle_, 1, 2, tab_reg);
    if (req == -1) {
      std::cout << "Pose read failed." << std::endl;
      std::cout << modbus_strerror(errno) << std::endl;
      pose_fail_status = true;
      pose = last_pose_;
      return;
    }
    pose = to_degree(combine_LSB_MSB(tab_reg[1], tab_reg[0]));
    // pose = degreesToRadians(pose);
    pose_fail_status = false;
    last_pose_ = pose;
    // read speed registers
    req = modbus_read_registers(dongle_, 5, 2, tab_reg);
    if (req == -1) {
      std::cout << "Speed read failed." << std::endl;
      std::cout << modbus_strerror(errno) << std::endl;
      speed_fail_status = true;
      speed = last_speed_;
      return;
    }
    speed = to_degree(combine_LSB_MSB(tab_reg[1], tab_reg[0]));
    speed_fail_status = false;
    last_speed_ = speed;
  } else {
    pose_fail_status = true;
    pose = last_pose_;
    speed_fail_status = true;
    speed = last_speed_;
  }
}

float PositalEncoderModbus::get_pose()
{
  return pose_;
}
float PositalEncoderModbus::get_speed()
{
  return speed_;
}
bool PositalEncoderModbus::get_pose_fail_status()
{
  return pose_fail_status_;
}
bool PositalEncoderModbus::get_speed_fail_status()
{
  return speed_fail_status_;
}
}  // namespace posital_encoder_modbus
