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

#include <memory>
#include <string>

#include <mbari_wec_template_cpp/control_policy.hpp>
#include <mbari_wec_template_cpp/controller.hpp>


Controller::Controller(const std::string & node_name)
: buoy_api::Interface<Controller>(node_name),
  policy_(std::make_unique<ControlPolicy>())
{
  this->set_params();

  // set packet rates from controllers here
  // controller defaults to publishing @ 10Hz
  // call these to set rate to 50Hz or provide argument for specific rate
  // this->set_sc_pack_rate();  // set SC publish rate to 50Hz
  // this->set_pc_pack_rate();  // set PC publish rate to 50Hz

  // Use this to set node clock to use sim time from /clock (from gazebo sim time)
  // Access node clock via this->get_clock() or other various time-related functions of rclcpp::Node
  // this->use_sim_time();
}

// To subscribe to any topic, simply declare & define the specific callback, e.g. power_callback
//
// // Callback for '/power_data' topic from Power Controller
// void power_callback(const buoy_interfaces::msg::PCRecord & data)
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

// Delete any unused callback
// Callback for '/ahrs_data' topic from XBowAHRS
void Controller::ahrs_callback(const buoy_interfaces::msg::XBRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}

// Callback for '/battery_data' topic from Battery Controller
void Controller::battery_callback(const buoy_interfaces::msg::BCRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}

// Callback for '/spring_data' topic from Spring Controller
void Controller::spring_callback(const buoy_interfaces::msg::SCRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}

// Callback for '/power_data' topic from Power Controller
void Controller::power_callback(const buoy_interfaces::msg::PCRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}

// Callback for '/trefoil_data' topic from Trefoil Controller
void Controller::trefoil_callback(const buoy_interfaces::msg::TFRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}

// Callback for '/powerbuoy_data' topic -- Aggregated data from all topics
void Controller::powerbuoy_callback(const buoy_interfaces::msg::PBRecord & /*data*/)
{
  // Update class variables, get control policy target, send commands, etc.
  // double target_value = policy_->target(data);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Controller>("controller"));
  rclcpp::shutdown();

  return 0;
}
