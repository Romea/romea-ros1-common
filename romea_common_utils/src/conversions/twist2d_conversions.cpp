//romea
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
void toRosMsg(const Twist2D & romea_twist2d,romea_common_msgs::Twist2D & ros_twist2d_msg)
{
  ros_twist2d_msg.linear_speeds.x = romea_twist2d.linearSpeeds.x();
  ros_twist2d_msg.linear_speeds.y = romea_twist2d.linearSpeeds.y();
  ros_twist2d_msg.angular_speed = romea_twist2d.angularSpeed;

  for(size_t n=0;n<9;++n)
  {
    ros_twist2d_msg.covariance[n]=romea_twist2d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const Twist2D & romea_twist_2d,
              romea_common_msgs::Twist2DStamped & ros_twist2d_msg)
{
  ros_twist2d_msg.header.stamp=stamp;
  ros_twist2d_msg.header.frame_id=frame_id;
  toRosMsg(romea_twist_2d,ros_twist2d_msg.twist);
}


//-----------------------------------------------------------------------------
void toRomea(const romea_common_msgs::Twist2D &ros_twist2d_msg,
             Twist2D & romea_twist2d)
{
  romea_twist2d.linearSpeeds.x()=ros_twist2d_msg.linear_speeds.x;
  romea_twist2d.linearSpeeds.y()=ros_twist2d_msg.linear_speeds.y;
  romea_twist2d.angularSpeed=ros_twist2d_msg.angular_speed;
  romea_twist2d.covariance=Eigen::Matrix3d(ros_twist2d_msg.covariance.data());
}

//-----------------------------------------------------------------------------
Twist2D toRomea(const romea_common_msgs::Twist2D & ros_twist2d_msg)
{
  Twist2D romea_twist2d;
  toRomea(ros_twist2d_msg,romea_twist2d);
  return romea_twist2d;
}


}// namespace

