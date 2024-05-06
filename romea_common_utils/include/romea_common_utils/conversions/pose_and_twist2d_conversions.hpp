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

#ifndef _romea_PoseAndTwist2DConversions_hpp_
#define _romea_PoseAndTwist2DConversions_hpp_

//romea
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>
#include "pose2d_conversions.hpp"
#include "twist2d_conversions.hpp"

//romea_ros_msg
#include <romea_common_msgs/PoseAndTwist2D.h>
#include <romea_common_msgs/PoseAndTwist2DStamped.h>

//ros
#include <tf/tf.h>

namespace romea
{


void to_ros_msg(const core::PoseAndTwist2D & romea_pose_and_twist2d,
                romea_common_msgs::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_ros_msg(const ros::Time & stamp,
                const std::string & frame_id,
                const core::PoseAndTwist2D & romea_pose_and_twist2d,
                romea_common_msgs::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped);

core::PoseAndTwist2D to_romea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg);

void to_romea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg,
              core::PoseAndTwist2D & romea_pose_and_twist2d);


}

#endif
