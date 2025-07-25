// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _romea_TimeConversion_hpp_
#define _romea_TimeConversion_hpp_

//romea
#include <romea_core_common/time/Time.hpp>

//ros
#include <ros/ros.h>

namespace romea {

   ros::Time to_ros_time(const core::Duration & duration);

   core::Duration to_romea_duration(const ros::Time & time);

   core::TimePoint to_romea_time(const ros::Time & time);

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
