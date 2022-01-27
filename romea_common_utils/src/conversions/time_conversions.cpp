#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
ros::Time to_ros_time(const Duration & duration)
{
  return ros::Time(durationToSecond(duration));
}

//-----------------------------------------------------------------------------
Duration to_romea_duration(const ros::Time & time)
{
  return Duration(time.toNSec());
}

}
