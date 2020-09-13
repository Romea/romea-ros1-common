#include "romea_common_utils/conversions/TimeConversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
ros::Time toROSTime(const Duration & duration)
{
  return ros::Time(durationToSecond(duration));
}

//-----------------------------------------------------------------------------
Duration toRomeaDuration(const ros::Time & time)
{
  return Duration(time.toNSec());
}

}
