#ifndef _romea_Position2DConversions_hpp_
#define _romea_Position2DConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Position2D.hpp>

//romea_ros_msg
#include <romea_common_msgs/Position2D.h>
#include <romea_common_msgs/Position2DStamped.h>

namespace romea
{


void to_ros_msg(const core::Position2D & romea_position2d,
              romea_common_msgs::Position2D & ros_position2d_msg);

void to_ros_msg(const ros::Time & stamp,
              const std::string & frame_id,
              const core::Position2D & position2D,
              romea_common_msgs::Position2DStamped & ros_position2d_stamped);


void to_romea(const romea_common_msgs::Position2D &msg,
             core::Position2D & position2d);

core::Position2D to_romea(const romea_common_msgs::Position2D &msg);

}// namespace

#endif
