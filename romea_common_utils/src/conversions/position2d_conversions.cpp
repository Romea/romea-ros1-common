//romea
#include "romea_common_utils/conversions/position2d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void toRosMsg(const Position2D & romea_position2d,
              romea_common_msgs::Position2D & ros_position2d_msg)
{
  ros_position2d_msg.x = romea_position2d.position.x();
  ros_position2d_msg.y = romea_position2d.position.y();

  for(size_t n=0;n<4;++n)
  {
    ros_position2d_msg.covariance[n]=romea_position2d.covariance(n);
  }

}

//-----------------------------------------------------------------------------
void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const Position2D & romea_position2d,
              romea_common_msgs::Position2DStamped & ros_position2d_stamped)
{
  ros_position2d_stamped.header.frame_id=frame_id;
  ros_position2d_stamped.header.stamp = stamp;
  toRosMsg(romea_position2d,ros_position2d_stamped.position);
}



//-----------------------------------------------------------------------------
void toRomea(const romea_common_msgs::Position2D & msg,
             Position2D & position2d)
{
  position2d.position.x()=msg.x;
  position2d.position.y()=msg.y;
  position2d.covariance=  Eigen::Matrix2d(msg.covariance.data());
}

//-----------------------------------------------------------------------------
Position2D toRomea(const romea_common_msgs::Position2D &msg)
{
  Position2D position2d;
  toRomea(msg,position2d);
  return position2d;
}

}// namespace

