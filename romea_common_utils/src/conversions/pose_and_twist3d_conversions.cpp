//romea
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void toRosOdomMsg(const ros::Time &stamp,
                  const PoseAndTwist3D & poseAndBodyTwist3D,
                  const std::string & frame_id,
                  const std::string & child_frame_id,
                  nav_msgs::Odometry & odom_msg)
{
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;
  toRosMsg(poseAndBodyTwist3D.pose,odom_msg.pose);
  toRosMsg(poseAndBodyTwist3D.twist,odom_msg.twist);
}


}// namespace
