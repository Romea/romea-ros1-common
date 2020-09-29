#ifndef _romea_TimeConversion_hpp_
#define _romea_TimeConversion_hpp_

//romea
#include <romea_common/time/Time.hpp>

//ros
#include <ros/ros.h>

namespace romea {

   ros::Time toROSTime(const romea::Duration & duration);

   romea::Duration toRomeaDuration(const ros::Time & time);

   template <typename Msg>
   Duration extractDuration(const Msg & msg)
   {
     auto time_stamp = ros::message_traits::timeStamp(msg);
     if(time_stamp)
     {
       return toRomeaDuration(*time_stamp);
     }
     else
     {
       return toRomeaDuration(ros::Time::now());
     }
   }
}

#endif
