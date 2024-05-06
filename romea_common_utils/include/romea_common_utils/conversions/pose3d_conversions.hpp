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

#ifndef _romea_Pose3DConversions_hpp_
#define _romea_Pose3DConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose3D.hpp>

//ros
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace romea
{


void toRosTransformMsg(const core::Pose3D & romea_pose_3d,
                       geometry_msgs::Transform & ros_transform_msg);


void toRosTransformMsg(const ros::Time &stamp,
                       const core::Pose3D &romea_pose_3d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::TransformStamped &tf_msg);

void to_ros_msg(const core::Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovariance & ros_pose_msg);

void to_ros_msg(const ros::Time & stamp,
              const std::string &frame_id,
              const core::Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovarianceStamped & ros_pose_msg);


void to_romea(const geometry_msgs::PoseWithCovariance & ros_pose_msg,
             core::Pose3D & romea_pose_3d);

core::Pose3D to_romea(const geometry_msgs::PoseWithCovariance & ros_pose_msg);

}

#endif
