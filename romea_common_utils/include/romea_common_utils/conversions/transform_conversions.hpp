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

#ifndef _romea_TransformConversions_hpp_
#define _romea_TransformConversions_hpp_

//ros
#include <geometry_msgs/Transform.h>


//eigen
#include <Eigen/Geometry>

namespace romea {

void to_romea(const geometry_msgs::Transform & tranform_msg,
             Eigen::Affine3d &eigen_transform);

void toRosTransformMsg(const Eigen::Affine3d &eigen_transform,
                       geometry_msgs::Transform & tranform_msg);

Eigen::Affine3d lookupTransformOnce(const std::string& target_frame,
                                    const std::string& source_frame,
                                    const ros::Time& time,
                                    const ros::Duration timeout);

}

#endif
