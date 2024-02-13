#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
ros::Time to_ros_time(const core::Duration & duration)
{
  return ros::Time(core::durationToSecond(duration));
}

//-----------------------------------------------------------------------------
core::Duration to_romea_duration(const ros::Time & time)
{
  return core::Duration(time.toNSec());
}

}
