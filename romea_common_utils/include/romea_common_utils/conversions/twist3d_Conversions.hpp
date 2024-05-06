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

#ifndef _romea_Twist3DConversions_hpp_
#define _romea_Twist3DConversions_hpp_


//romea
#include <romea_core_common/geometry/Twist3D.hpp>

//ros
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace romea
{

void to_ros_msg(const core::Twist3D & romea_twist_3d,
              geometry_msgs::TwistWithCovariance & ros_twist_msg);

void to_ros_msg(const ros::Time & stamp,
              const std::string & frame_id,
              const core::Twist3D & romea_twist_3d,
              geometry_msgs::TwistWithCovarianceStamped & ros_twist3d_msg);

void to_romea(const geometry_msgs::TwistWithCovariance & ros_twist_msg,
             core::Twist3D & romea_twist3d);

core::Twist3D to_romea(const geometry_msgs::TwistWithCovariance & ros_twist_msg);


}// namespace

#endif
