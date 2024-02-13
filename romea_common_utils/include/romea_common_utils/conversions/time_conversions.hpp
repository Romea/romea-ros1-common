#ifndef _romea_TimeConversion_hpp_
#define _romea_TimeConversion_hpp_

//romea
#include <romea_core_common/time/Time.hpp>

//ros
#include <ros/ros.h>

namespace romea {

   ros::Time to_ros_time(const core::Duration & duration);

   core::Duration to_romea_duration(const ros::Time & time);

   template <typename Msg>
   core::Duration extractDuration(const Msg & msg)
   {
     auto time_stamp = ros::message_traits::timeStamp(msg);
     if(time_stamp)
     {
       return to_romea_duration(*time_stamp);
     }
     else
     {
       return to_romea_duration(ros::Time::now());
     }
   }
}

#endif
