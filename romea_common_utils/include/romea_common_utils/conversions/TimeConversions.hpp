#ifndef _romea_TimeConversion_hpp_
#define _romea_TimeConversion_hpp

//romea
#include <romea_common/time/Time.hpp>

//ros
#include <ros/time.h>

namespace romea {

   ros::Time toROSTime(const romea::Duration & duration);

   romea::Duration toRomeaDuration(const ros::Time & time);
}

#endif
