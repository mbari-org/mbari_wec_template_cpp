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

#ifndef MBARI_WEC_TEMPLATE_CPP__CONTROL_POLICY_HPP_
#define MBARI_WEC_TEMPLATE_CPP__CONTROL_POLICY_HPP_

/********************************************************
/ User-space to define control policy and param loading /
********************************************************/

#include <mbari_wec_template_cpp/controller.hpp>


struct ControlPolicy
{
  // declare/init any parameter variables here
  double foo{1.0};
  double bar{10.0*foo};

  ControlPolicy()
  : foo{1.0},
    bar{10.0*foo}
  {
    update_params();
  }

  // Update dependent variables after reading in params
  void update_params()
  {
    bar = 10.0*foo;
  }

  // Modify function inputs as desired
  // Calculate target value from feedback inputs
  double target(
    const double & /*some*/,
    const double & /*feedback*/,
    const double & /*values*/)
  {

    // secret sauce

    return 0.0;  // obviously, modify to return proper target value
  }
};

// Use ROS2 declare_parameter and get_parameter to set policy params
void Controller::set_params()
{
  this->declare_parameter("foo", policy_->foo);
  policy_->foo = this->get_parameter("foo").as_double();

  // recompute any dependent variables
  policy_->update_params();
}
#endif  // MBARI_WEC_TEMPLATE_CPP__CONTROL_POLICY_HPP_
