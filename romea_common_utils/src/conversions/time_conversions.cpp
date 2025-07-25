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

#include "romea_common_utils/conversions/time_conversions.hpp"
#include <romea_core_common/time/Time.hpp>

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

core::TimePoint to_romea_time(const ros::Time & time)
{
  return core::TimePoint(to_romea_duration(time));
}

}
