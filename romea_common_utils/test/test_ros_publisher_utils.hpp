//eigen
#include <Eigen/Core>

//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>

inline void toRosMsg(const std::string & data,
                     std_msgs::String & msg)
{
  msg.data=data;
}

inline void toRosOdomMsg(const ros::Time & t,
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

inline void toRosMsg(const ros::Time & stamp,
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
