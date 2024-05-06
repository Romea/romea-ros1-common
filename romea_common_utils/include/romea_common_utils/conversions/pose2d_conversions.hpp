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

#ifndef _romea_Pose2DConversions_hpp_
#define _romea_Pose2DConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose2D.hpp>

//romea_ros_msg
#include <romea_common_msgs/Pose2DStamped.h>

//ros
#include <geometry_msgs/TransformStamped.h>

namespace romea
{

void to_ros_msg(const core::Pose2D & romea_pose2d,
                romea_common_msgs::Pose2D & ros_pose2d_msg);

void to_ros_msg(const ros::Time & stamp,
                const std::string & frame_id,
                const core::Pose2D & romea_pose2d,
                romea_common_msgs::Pose2DStamped & ros_pose2d_msg);

void toRosTransformMsg(const core::Pose2D & romea_pose2d,
                       geometry_msgs::Transform & ros_transform_msg);


void toRosTransformMsg(const ros::Time &stamp,
                       const core::Pose2D &romea_pose_2d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::TransformStamped &ros_transform_msg);

void to_romea(const romea_common_msgs::Pose2D &msg,
              core::Pose2D & romea_pose_2d);

core::Pose2D to_romea(const romea_common_msgs::Pose2D &msg);

//void to_romea(const romea_localisation_msgs::Pose2DStamped &msg,
//             core::Pose2D::Stamped & romea_pose_2d_stamped,
//             std::string & frame_id);

}// namespace

#endif
