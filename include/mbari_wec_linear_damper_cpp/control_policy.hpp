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

#ifndef MBARI_WEC_LINEAR_DAMPER_CPP__CONTROL_POLICY_HPP_
#define MBARI_WEC_LINEAR_DAMPER_CPP__CONTROL_POLICY_HPP_

/********************************************************
/ User-space to define control policy and param loading /
********************************************************/

#include <algorithm>
#include <vector>

#include <mbari_wec_linear_damper_cpp/controller.hpp>

// interp1d for rpm->winding current
#include <simple_interp/interp1d.hpp>


/* Simple Linear Damper Control Policy.
     Implements a simple linear damper controller for the piston in the WEC
     Power-Take-Off (PTO). Given motor RPM, outputs desired motor winding current (interpolated
     from RPM->Torque lookup table) to resist piston velocity. Configurable gains
     (scale/retract factor) are applied before output.
*/
struct ControlPolicy
{
  // declare/init any parameter variables here
  double Torque_constant;  // N-m/Amps
  std::vector<double> N_Spec;  // RPM
  std::vector<double> Torque_Spec;  // N-m
  std::vector<double> I_Spec;  // Amps

  // interpolator for rpm -> winding current
  simple_interp::Interp1d winding_current;

  ControlPolicy()
  : Torque_constant(0.438F),
    N_Spec{0.0F, 300.0F, 600.0F, 1000.0F, 1700.0F, 4400.0F, 6790.0F},
    Torque_Spec{0.0F, 0.0F, 0.8F, 2.9F, 5.6F, 9.8F, 16.6F},
    I_Spec(Torque_Spec.size(), 0.0F),
    winding_current(N_Spec, I_Spec)
  {
    update_params();
  }

  // Update dependent variables after reading in params
  void update_params()
  {
    std::transform(
      Torque_Spec.cbegin(), Torque_Spec.cend(),
      I_Spec.begin(),
      [tc = Torque_constant](const double & ts) {return ts / tc;});

    winding_current.update(N_Spec, I_Spec);
  }

  // Calculate target value from feedback inputs
  double target(
    const double & rpm,
    const double & scale_factor,
    const double & retract_factor)
  {
    double N = fabs(rpm);
    double I = winding_current.eval(N);

    // apply damping gain
    I *= scale_factor;

    // Hysteresis due to gravity / wave assist
    if (rpm > 0.0F) {
      I *= -retract_factor;
    }

    return I;
  }
};

// Helper function to print policy parameters
std::ostream & operator<<(std::ostream & os, const ControlPolicy & policy)
{
  os << "ControlPolicy:" << std::endl;

  os << "\tTorque_constant: " << policy.Torque_constant << std::endl;

  os << "\tN_Spec: " << std::flush;
  std::copy(policy.N_Spec.cbegin(), policy.N_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tTorque_Spec: " << std::flush;
  std::copy(
    policy.Torque_Spec.cbegin(),
    policy.Torque_Spec.cend(),
    std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tI_Spec: " << std::flush;
  std::copy(policy.I_Spec.cbegin(), policy.I_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  return os;
}

// Use ROS2 declare_parameter and get_parameter to set policy params
void Controller::set_params()
{
  this->declare_parameter("torque_constant", policy_->Torque_constant);
  policy_->Torque_constant = this->get_parameter("torque_constant").as_double();

  this->declare_parameter(
    "n_spec", std::vector<double>(
      policy_->N_Spec.begin(),
      policy_->N_Spec.end()));
  std::vector<double> temp_double_arr = this->get_parameter("n_spec").as_double_array();
  policy_->N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  this->declare_parameter(
    "torque_spec", std::vector<double>(
      policy_->Torque_Spec.begin(),
      policy_->Torque_Spec.end()));
  temp_double_arr = this->get_parameter("torque_spec").as_double_array();
  policy_->Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  // recompute any dependent variables
  policy_->update_params();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), *policy_);
}

#endif  // MBARI_WEC_LINEAR_DAMPER_CPP__CONTROL_POLICY_HPP_
