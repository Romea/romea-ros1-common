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

#ifndef _romea_GeometryConversions_hpp_
#define _romea_GeometryConversions_hpp_

//ros
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <romea_common_msgs/PoseAndTwist2DStamped.h>

//eigen
#include <Eigen/Geometry>

//romea



namespace romea
{

void to_romea(const geometry_msgs::Vector3 & position_msg,
              Eigen::Vector3d & eigen_position);

void to_ros_msg(const Eigen::Vector3d & eigen_position,
                geometry_msgs::Vector3 & position_msg);

void to_romea(const geometry_msgs::Quaternion & quaternion_msg,
              Eigen::Quaterniond & eigen_quaternion);

void to_ros_msg(const Eigen::Quaterniond & eigen_quaternion,
                geometry_msgs::Quaternion &quaternion_msg);

void to_romea(const geometry_msgs::Quaternion & quaternion_msg,
              Eigen::Matrix3d & eigen_rotation_matrix);

void to_ros_msg(const Eigen::Matrix3d & eigen_rotation_matrix,
                geometry_msgs::Quaternion &quaternion_msg);

}

#endif
