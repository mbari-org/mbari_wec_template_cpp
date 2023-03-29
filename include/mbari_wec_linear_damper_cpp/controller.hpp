// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MBARI_WEC_LINEAR_DAMPER_CPP__CONTROLLER_HPP_
#define MBARI_WEC_LINEAR_DAMPER_CPP__CONTROLLER_HPP_

#include <memory>
#include <string>

#include <buoy_api/interface.hpp>

// forward declare
struct ControlPolicy;  // defined in control_policy.hpp

// Use CRTP
class Controller final : public buoy_api::Interface<Controller>
{
public:
  explicit Controller(const std::string & node_name);
  ~Controller() = default;

private:
  friend CRTP;  // syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  void set_params() final;  // defined in control_policy.hpp

  // To subscribe to any topic, simply declare & define the specific callback, e.g. power_callback
  //
  // // Callback for '/power_data' topic from Power Controller
  // void power_callback(const buoy_interfaces::msg::PCRecord &)
  // {
  //   // get target value from control policy
  //   double wind_curr = policy_->target(data.rpm, data.scale, data.retract);
  //
  //   auto future = this->send_pc_wind_curr_command(wind_curr);
  // }

  // Available commands to send within any callback:
  // this->send_pump_command(duration_mins);
  // this->send_valve_command(duration_sec);
  // this->send_pc_wind_curr_command(wind_curr_amps);
  // this->send_pc_bias_curr_command(bias_curr_amps);
  // this->send_pc_scale_command(scale_factor);
  // this->send_pc_retract_command(retract_factor);

  // Callback for '/power_data' topic from Power Controller
  void power_callback(const buoy_interfaces::msg::PCRecord & data);

  std::unique_ptr<ControlPolicy> policy_;
};

#endif  // MBARI_WEC_LINEAR_DAMPER_CPP__CONTROLLER_HPP_
