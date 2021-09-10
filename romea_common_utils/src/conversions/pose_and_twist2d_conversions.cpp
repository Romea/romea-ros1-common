#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea {

//-----------------------------------------------------------------------------
void toRosMsg(const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::PoseAndTwist2D & ros_pose_and_twist2d_msg)
{
  toRosMsg(romea_pose_and_twist2d.pose,ros_pose_and_twist2d_msg.pose);
  toRosMsg(romea_pose_and_twist2d.twist,ros_pose_and_twist2d_msg.twist);
}

//-----------------------------------------------------------------------------
void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped)
{
  ros_pose_and_twist2d_msg_stamped.header.stamp=stamp;
  ros_pose_and_twist2d_msg_stamped.header.frame_id=frame_id;
  toRosMsg(romea_pose_and_twist2d.pose,ros_pose_and_twist2d_msg_stamped.pose);
  toRosMsg(romea_pose_and_twist2d.twist,ros_pose_and_twist2d_msg_stamped.twist);
}

//-----------------------------------------------------------------------------
PoseAndTwist2D toRomea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg)
{
  PoseAndTwist2D romea_pose_and_twist2d;
  toRomea(ros_pose_and_twist2d_msg,romea_pose_and_twist2d);
  return romea_pose_and_twist2d;
}

//-----------------------------------------------------------------------------
void toRomea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg,
             PoseAndTwist2D & romea_pose_and_twist2d)
{
  toRomea(ros_pose_and_twist2d_msg.pose,romea_pose_and_twist2d.pose);
  toRomea(ros_pose_and_twist2d_msg.twist,romea_pose_and_twist2d.twist);
}

}
