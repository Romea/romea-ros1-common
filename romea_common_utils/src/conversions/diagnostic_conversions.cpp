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

#include "romea_common_utils/conversions/diagnostic_conversions.hpp"

namespace
{

}

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_diagnostic_msg(const std::string & diagnoctic_name,
                           const std::string & hardware_id,
                           const core::DiagnosticReport & report,
                           diagnostic_msgs::DiagnosticStatus & msg)
{
  msg.level=static_cast<int>(worseStatus(report.diagnostics));
  msg.hardware_id=hardware_id;
  msg.name = diagnoctic_name;

  for(const auto & diagnostic : report.diagnostics)
  {
    if(diagnostic.status!=core::DiagnosticStatus::OK)
    {
      msg.message+=diagnostic.message+" ";
    }
  }

  for(const auto & [info_name,info_status] : report.info)
  {
    diagnostic_msgs::KeyValue key;
    key.key=info_name;
    key.value=info_status;
    msg.values.push_back(key);
  }
}


}
