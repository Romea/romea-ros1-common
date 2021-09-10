#ifndef _romea_Twist2DConversions_hpp_
#define _romea_Twist2DConversions_hpp_

//romea
#include <romea_common/geometry/Twist2D.hpp>

//romea_ros_msg
#include <romea_common_msgs/Twist2DStamped.h>

namespace romea
{

void toRosMsg(const Twist2D & romea_twist_2d,
              romea_common_msgs::Twist2D & ros_twist2d_msg);

void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const Twist2D & romea_twist_2d,
              romea_common_msgs::Twist2DStamped & ros_twist2d_msg);

void toRomea(const romea_common_msgs::Twist2D & ros_twist2d_msg,
             Twist2D & romea_twist2d);

Twist2D toRomea(const romea_common_msgs::Twist2D & ros_twist2d_msg);

}// namespace

#endif
