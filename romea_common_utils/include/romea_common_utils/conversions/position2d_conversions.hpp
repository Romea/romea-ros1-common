#ifndef _romea_Position2DConversions_hpp_
#define _romea_Position2DConversions_hpp_

//std
#include <string>

//romea
#include <romea_common/time/Time.hpp>
#include <romea_common/geometry/Position2D.hpp>

//romea_ros_msg
#include <romea_common_msgs/Position2D.h>
#include <romea_common_msgs/Position2DStamped.h>

namespace romea
{


void toRosMsg(const Position2D & romea_position2d,
              romea_common_msgs::Position2D & ros_position2d_msg);

void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const Position2D & position2D,
              romea_common_msgs::Position2DStamped & ros_position2d_stamped);


void toRomea(const romea_common_msgs::Position2D &msg,
             Position2D & position2d);

Position2D toRomea(const romea_common_msgs::Position2D &msg);

}// namespace

#endif
