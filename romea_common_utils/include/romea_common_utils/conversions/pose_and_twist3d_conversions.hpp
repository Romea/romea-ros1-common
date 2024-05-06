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

#ifndef _romea_OdomConversions_hpp_
#define _romea_OdomConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include "pose3d_conversions.hpp"
#include "twist3d_Conversions.hpp"

//romea_ros_msg
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

namespace romea
{

void  to_ros_odom_msg(const ros::Time &stamp,
                      const core::PoseAndTwist3D & poseAndBodyTwist3D,
                      const std::string & frame_id,
                      const std::string & child_frame_id,
                      nav_msgs::Odometry & odom_msg);


}// namespace

#endif
