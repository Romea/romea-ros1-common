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

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// local
#include "romea_common_utils/publishers/publisher.hpp"

namespace romea
{
namespace ros1
{

template<typename DataType, typename MsgType>
class DataPublisher : public Publisher<DataType, MsgType>
{
private:
  using Base = Publisher<DataType, MsgType>;

public:
  using Base::Publisher;

  virtual ~DataPublisher() = default;

  void publish(const DataType & data)
  {
    auto msg = std::make_unique<MsgType>();
    to_ros_msg(data, *msg.get());
    this->publish_message_(std::move(msg));
  }
};

template<typename DataType, typename MsgType>
std::unique_ptr<DataPublisher<DataType, MsgType>> make_data_publisher(
  ros::NodeHandle & nh,
  const std::string & topic_name,
  std::uint32_t queue_size,
  const bool & activated)
{
  using Publisher = DataPublisher<DataType, MsgType>;
  return std::make_unique<Publisher>(nh, topic_name, queue_size, activated);
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_
