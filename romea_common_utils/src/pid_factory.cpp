// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#include "romea_common_utils/pid_factory.hpp"
#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{

std::unique_ptr<core::PID> make_pid(ros::NodeHandle &nh)
{
  return std::make_unique<core::PID>(load_param<double>(nh,"kp"),
                               load_param<double>(nh,"ki"),
                               load_param<double>(nh,"kd"),
                               load_param<double>(nh,"imin"),
                               load_param<double>(nh,"imax"),
                               load_param<double>(nh,"error_epsilon"));
}

}

