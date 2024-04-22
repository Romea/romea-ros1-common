// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_

// ros
#include <ros/file_log.h>
#include <ros/node_handle.h>

// local
#include "romea_common_utils/params/ros_param.hpp"

namespace romea
{
namespace ros1
{

//-----------------------------------------------------------------------------
inline bool get_debug(const ros::NodeHandle & nh)
{
  return load_param_or<bool>(nh, "debug", false);
}

//-----------------------------------------------------------------------------
inline std::string get_log_directory(const ros::NodeHandle & nh)
{
  return load_param_or<std::string>(nh, "log_directory", ros::file_log::getLogDirectory());
}

//-----------------------------------------------------------------------------
inline std::string get_log_filename(const ros::NodeHandle & nh, const std::string & log_name = "")
{
  std::string filename;
  if (get_debug(nh)) {
    std::string ns = nh.getNamespace();

    filename = ns == "/" ? "" : ns;

    if (!log_name.empty()) {
      filename += "/" + log_name;
    }

    filename += "/debug.csv";

    std::replace_copy(
      std::begin(filename) + 1, std::end(filename), std::begin(filename) + 1, '/', '_');

    filename = get_log_directory(nh) + filename;
  }
  return filename;
}

//-----------------------------------------------------------------------------
inline std::string get_base_footprint_frame_id(const ros::NodeHandle & nh)
{
  return load_param_or<std::string>(nh, "base_footprint_frame_id", "base_footprint");
}

//-----------------------------------------------------------------------------
inline std::string get_odom_frame_id(const ros::NodeHandle & nh)
{
  return load_param_or<std::string>(nh, "odom_frame_id", "odom");
}

//-----------------------------------------------------------------------------
inline std::string get_map_frame_id(const ros::NodeHandle & nh)
{
  return load_param_or<std::string>(nh, "map_frame_id", "map");
}

//-----------------------------------------------------------------------------
inline int get_publish_rate(const ros::NodeHandle & nh, int default_value)
{
  return load_param_or<int>(nh, "publish_rate", default_value);
}

//-----------------------------------------------------------------------------
inline int get_publish_rate(const ros::NodeHandle & nh)
{
  return load_param<int>(nh, "publish_rate");
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_
