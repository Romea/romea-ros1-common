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

//eigen
#include <Eigen/Core>

//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>

inline void to_ros_msg(const std::string & data,
                     std_msgs::String & msg)
{
  msg.data=data;
}

inline void to_ros_odom_msg(const ros::Time & t,
                         const nav_msgs::Odometry & data,
                         const std::string & frame_id,
                         const std::string & child_frame_id,
                         nav_msgs::Odometry & msg)
{
  msg.header.stamp=t;
  msg.header.frame_id=frame_id;
  msg.child_frame_id=child_frame_id;
  msg.pose=data.pose;
  msg.twist=data.twist;
}

inline void to_ros_msg(const ros::Time & stamp,
                     const std::string & frame_id,
                     const Eigen::Vector3d & data,
                     geometry_msgs::PointStamped & msg)
{
  msg.header.stamp=stamp;
  msg.header.frame_id=frame_id;
  msg.point.x=data.x();
  msg.point.y=data.y();
  msg.point.z=data.z();
}


template <typename MsgType>
struct AnyHelper
{
  AnyHelper():
    data()
  {

  }

  void cb(const typename MsgType::ConstPtr & msg)
  {
    data=*msg;
  }

  MsgType data;
};
