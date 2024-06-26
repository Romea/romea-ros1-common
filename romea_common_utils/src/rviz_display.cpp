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

#include "romea_common_utils/rviz_display.hpp"
#include <tf2_eigen/tf2_eigen.h>

namespace {

//-----------------------------------------------------------------------------
inline  Eigen::Isometry3d computeRobotPose(const romea::core::Pose2D & bodyPose2D,
                                           const double & positionAlongZBodyAxis)
{
  return Eigen::Translation3d(bodyPose2D.position.x(),
                              bodyPose2D.position.y(),
                              positionAlongZBodyAxis)*
      Eigen::AngleAxisd(bodyPose2D.yaw,Eigen::Vector3d::UnitZ());

}

//-----------------------------------------------------------------------------
inline  Eigen::Isometry3d computeEllipsePose(const romea::core::Ellipse & ellipse,
                                             const double & positionAlongZBodyAxis)
{
  return Eigen::Translation3d(ellipse.getCenterPosition().x(),
                              ellipse.getCenterPosition().y(),
                              positionAlongZBodyAxis)*
      Eigen::AngleAxisd(ellipse.getOrientation(), Eigen::Vector3d::UnitZ());
}

}

namespace romea {


//-----------------------------------------------------------------------------
void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const core::Pose2D & bodyPose2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis,
             double scaleAlongZBodyAxis,
             double sigma)
{

  core::Ellipse ellipse= uncertaintyEllipse(bodyPose2D,sigma);

  geometry_msgs::Vector3 scale;
  scale.x = ellipse.getMajorRadius();
  scale.y = ellipse.getMinorRadius();
  scale.z = scaleAlongZBodyAxis;

  geometry_msgs::Pose ellipse_pose_msg =
      tf2::toMsg(computeEllipsePose(ellipse,positionAlongZBodyAxis));
  rvizVisualTool.publishSphere(ellipse_pose_msg,
                               rvizVisualTool.getColor(color),
                               scale);

  geometry_msgs::Pose robot_pose_msg=
      tf2::toMsg(computeRobotPose(bodyPose2D,positionAlongZBodyAxis));
  rvizVisualTool.publishAxis(robot_pose_msg,1);

}

//-----------------------------------------------------------------------------
void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const core::Position2D & bodyPosition2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis,
             double scaleAlongZBodyAxis,
             double sigma)
{

  core::Ellipse ellipse= uncertaintyEllipse(bodyPosition2D,sigma);

  geometry_msgs::Vector3 scale;
  scale.x = ellipse.getMajorRadius();
  scale.y = ellipse.getMinorRadius();
  scale.z = scaleAlongZBodyAxis;

  geometry_msgs::Pose ellipse_pose_msg=
      tf2::toMsg(computeEllipsePose(ellipse,positionAlongZBodyAxis));
  rvizVisualTool.publishSphere(ellipse_pose_msg,
                               rvizVisualTool.getColor(color),scale);

}




}
