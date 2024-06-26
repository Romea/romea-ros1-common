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

//romea
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_common_utils/conversions/transform_conversions.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const core::Pose2D & romea_pose2d,
                romea_common_msgs::Pose2D & ros_pose2d_msg)
{
  ros_pose2d_msg.position.x = romea_pose2d.position.x();
  ros_pose2d_msg.position.y = romea_pose2d.position.y();
  ros_pose2d_msg.yaw = romea_pose2d.yaw;

  for(size_t n=0;n<9;++n)
  {
    ros_pose2d_msg.covariance[n]=romea_pose2d.covariance(n);
  }

}

//-----------------------------------------------------------------------------
void to_ros_msg(const ros::Time &stamp,
                const std::string & frame_id,
                const core::Pose2D & romea_pose2d,
                romea_common_msgs::Pose2DStamped & ros_pose2d_stamped_msg)
{
  ros_pose2d_stamped_msg.header.frame_id=frame_id;
  ros_pose2d_stamped_msg.header.stamp = stamp;
  to_ros_msg(romea_pose2d,ros_pose2d_stamped_msg.pose);
}

//-----------------------------------------------------------------------------
void toRosTransformMsg(const core::Pose2D & romea_pose_2d, geometry_msgs::Transform  & ros_transform_msg)
{
  ros_transform_msg.translation.x=romea_pose_2d.position.x();
  ros_transform_msg.translation.y=romea_pose_2d.position.y();
  ros_transform_msg.translation.z=0;

  Eigen::Quaterniond q(Eigen::AngleAxisd(romea_pose_2d.yaw,Eigen::Vector3d::UnitZ()));
  ros_transform_msg.rotation.x=q.x();
  ros_transform_msg.rotation.y=q.y();
  ros_transform_msg.rotation.z=q.z();
  ros_transform_msg.rotation.w=q.w();
}



//-----------------------------------------------------------------------------
void toRosTransformMsg(const ros::Time &stamp,
                       const core::Pose2D & romea_pose_2d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::TransformStamped &ros_transform_msg)
{
  ros_transform_msg.header.stamp=stamp;
  ros_transform_msg.header.frame_id=frame_id;
  ros_transform_msg.child_frame_id = child_frame_id;
  toRosTransformMsg(romea_pose_2d,ros_transform_msg.transform);
}


//-----------------------------------------------------------------------------
void to_romea(const romea_common_msgs::Pose2D &msg,
              core::Pose2D & romea_pose_2d)
{
  romea_pose_2d.position.x()=msg.position.x;
  romea_pose_2d.position.y()=msg.position.y;
  romea_pose_2d.yaw=msg.yaw;
  romea_pose_2d.covariance=Eigen::Matrix3d(msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::Pose2D to_romea(const romea_common_msgs::Pose2D &msg)
{
  core::Pose2D romea_pose_2d;
  to_romea(msg,romea_pose_2d);
  return romea_pose_2d;
}

////-----------------------------------------------------------------------------
//void to_romea(const romea_common_msgs::Pose2DStamped &msg,
//             Pose2D::Stamped & romea_pose_2d_stamped,
//             std::string & frame_id)
//{
//  frame_id = msg.header.frame_id;
//  to_romea(msg.pose,romea_pose_2d_stamped.data);
//  romea_pose_2d_stamped.stamp=to_romea_duration(msg.header.stamp);
//}

}// namespace

