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
#include "romea_common_utils/conversions/pose3d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_core_common/math/EulerAngles.hpp>

//ros
#include <eigen_conversions/eigen_msg.h>

namespace romea
{


//-----------------------------------------------------------------------------
void toRosTransformMsg(const core::Pose3D & romea_pose_3d,
                       geometry_msgs::Transform & ros_transform_msg)
{
  ros_transform_msg.translation.x=romea_pose_3d.position.x();
  ros_transform_msg.translation.y=romea_pose_3d.position.y();
  ros_transform_msg.translation.z=romea_pose_3d.position.z();

  Eigen::Quaterniond q(core::eulerAnglesToRotation3D(romea_pose_3d.orientation));

  ros_transform_msg.rotation.x=q.x();
  ros_transform_msg.rotation.y=q.y();
  ros_transform_msg.rotation.z=q.z();
  ros_transform_msg.rotation.w=q.w();
}

//-----------------------------------------------------------------------------
void toRosTransformMsg(const ros::Time &stamp,
                       const core::Pose3D &romea_pose_3d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::TransformStamped &ros_transform_stamped_msg)
{
  ros_transform_stamped_msg.header.stamp=stamp;
  ros_transform_stamped_msg.header.frame_id=frame_id;
  ros_transform_stamped_msg.child_frame_id = child_frame_id;
  toRosTransformMsg(romea_pose_3d,ros_transform_stamped_msg.transform);
}

//-----------------------------------------------------------------------------
void to_ros_msg(const core::Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovariance & ros_pose_msg)
{
  const auto & position =romea_pose_3d.position;
  const auto & orientation =romea_pose_3d.orientation;
  const auto & covariance = romea_pose_3d.covariance;

  ros_pose_msg.pose.position.x=position.x();
  ros_pose_msg.pose.position.y=position.y();
  ros_pose_msg.pose.position.z=position.z();

  Eigen::Quaterniond q(core::eulerAnglesToRotation3D(orientation));

  ros_pose_msg.pose.orientation.x = q.x();
  ros_pose_msg.pose.orientation.y = q.y();
  ros_pose_msg.pose.orientation.z = q.z();
  ros_pose_msg.pose.orientation.w = q.w();

  for(size_t n=0;n<36;++n)
  {
    ros_pose_msg.covariance[n]=covariance(n);
  }

}

//-----------------------------------------------------------------------------
void to_ros_msg(const ros::Time & stamp,
              const std::string & frame_id,
              const core::Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovarianceStamped & ros_pose_msg)
{
  ros_pose_msg.header.stamp=stamp;
  ros_pose_msg.header.frame_id=frame_id;
  to_ros_msg(romea_pose_3d,ros_pose_msg.pose);
}

//-----------------------------------------------------------------------------
void to_romea(const geometry_msgs::PoseWithCovariance & ros_pose_msg,
             core::Pose3D & romea_pose_3d)
{
  romea_pose_3d.position.x()=ros_pose_msg.pose.position.x;
  romea_pose_3d.position.y()=ros_pose_msg.pose.position.y;
  romea_pose_3d.position.z()=ros_pose_msg.pose.position.z;;

  Eigen::Quaterniond q(ros_pose_msg.pose.orientation.w,
                       ros_pose_msg.pose.orientation.x,
                       ros_pose_msg.pose.orientation.y,
                       ros_pose_msg.pose.orientation.z);

  romea_pose_3d.orientation=core::rotation3DToEulerAngles(q.toRotationMatrix());
  romea_pose_3d.covariance=Eigen::Matrix6d(ros_pose_msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::Pose3D to_romea(const geometry_msgs::PoseWithCovariance & ros_pose_msg)
{
  core::Pose3D romea_pose_3d;
  to_romea(ros_pose_msg,romea_pose_3d);
  return romea_pose_3d;
}

}
