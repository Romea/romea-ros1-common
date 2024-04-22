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

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_

// local
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/publishers/stamped_publisher.hpp"

namespace romea
{
namespace ros1
{

template<typename DataType, typename MsgType>
class StampedDataPublisher : public StampedPublisher<DataType, MsgType>
{
private:
  using Base = StampedPublisher<DataType, MsgType>;

public:
  StampedDataPublisher(
    ros::NodeHandle & nh,
    const std::string & topic_name,
    const std::string & frame_id,
    std::uint32_t queue_size,
    const bool & activated)
  : Base(nh, topic_name, queue_size, activated), frame_id_(frame_id)
  {
    assert(!frame_id.empty());
  }

  void publish(const ros::Time & stamp, const DataType & data) override
  {
    auto msg = std::make_unique<MsgType>();
    to_ros_msg(stamp, frame_id_, data, *msg.get());
    this->publish_message_(std::move(msg));
  }

  void publish(const core::Duration & duration, const DataType & data) override
  {
    publish(to_ros_time(duration), data);
  }

private:
  std::string frame_id_;
};

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType>
std::unique_ptr<StampedDataPublisher<DataType, MsgType>> make_stamped_data_publisher(
  ros::NodeHandle & nh,
  const std::string & topic_name,
  const std::string & frame_id,
  std::uint32_t queue_size,
  const bool & activated)
{
  using Publisher = StampedDataPublisher<DataType, MsgType>;
  return std::make_unique<Publisher>(nh, topic_name, frame_id, queue_size, activated);
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_DATA_PUBLISHER_HPP_
