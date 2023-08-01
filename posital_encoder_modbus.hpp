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

#ifndef POSITAL_ENCODER_MODBUS__POSITAL_ENCODER_MODBUS_HPP_
#define POSITAL_ENCODER_MODBUS__POSITAL_ENCODER_MODBUS_HPP_
#define POSITAL_ENCODER_MODBUS_PUBLIC __attribute__((visibility("default")))
#define POSITAL_ENCODER_MODBUS_LOCAL __attribute__((visibility("hidden")))

#include <modbus/modbus.h>

#include <cstdint>
#include <string>
#include <vector>
namespace posital_encoder_modbus
{

  class POSITAL_ENCODER_MODBUS_PUBLIC PositalEncoderModbus
  {
  private:
    modbus_t *dongle_;
    float pose_ = 0.0f;
    float speed_ = 0.0f;
    bool pose_fail_status_ = false;
    bool speed_fail_status_ = false;

  public:
    PositalEncoderModbus();
    PositalEncoderModbus(std::string device, int baud, char parity, int data_bit, int stop_bit, int debug);
    void read_data(float &speed, bool &speed_fail_status, float &pose, bool &pose_fail_status);
    void read_data();
    bool init_encoder_motor(int slave_id, int speed_mode_value, int speed_filter_value);
    float last_pose_ = 0.0f;
    float last_speed_ = 0.0f;
    float get_pose();
    float get_speed();
    bool get_pose_fail_status();
    bool get_speed_fail_status();
  };

} // namespace posital_encoder_modbus

#endif // POSITAL_ENCODER_MODBUS__POSITAL_ENCODER_MODBUS_HPP_
