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

#ifndef _romea_DiagnosticConversions_hpp_
#define _romea_DiagnosticConversions_hpp_

//romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

//ros
#include <diagnostic_msgs/DiagnosticArray.h>

namespace romea
{
void to_ros_diagnostic_msg(const std::string & diagnostic_name,
                           const std::string & hardware_id,
                           const core::DiagnosticReport & report,
                           diagnostic_msgs::DiagnosticStatus &msg);

}

#endif
