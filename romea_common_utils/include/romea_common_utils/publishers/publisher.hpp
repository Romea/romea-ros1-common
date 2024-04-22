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

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// ros
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace romea
{
namespace ros1
{

template<typename DataType>
class PublisherBase
{
public:
  virtual ~PublisherBase() = default;
  virtual std::string get_topic_name() const = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual bool is_activated() = 0;
  virtual void publish(const DataType & data) = 0;
};

template<typename DataType, typename MsgType>
class Publisher : public PublisherBase<DataType>
{
public:
  Publisher(
    ros::NodeHandle & nh, const std::string & topic_name, std::uint32_t queue_size, bool activated)
  : is_activated_(activated), pub_(nh.advertise<MsgType>(topic_name, queue_size))
  {
  }

  std::string get_topic_name() const override { return pub_.getTopic(); }

  void activate() override { is_activated_.store(true); }

  void deactivate() override { is_activated_.store(false); }

  bool is_activated() override { return is_activated_.load(); }

protected:
  void publish_message_(const boost::shared_ptr<MsgType> & message)
  {
    if (!is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    pub_.publish(message);
  }

  void publish_message_(const MsgType & message)
  {
    if (!is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    pub_.publish(message);
  }

  void log_publisher_not_enabled()
  {
    ROS_WARN_ONCE(
      "Trying to publish message on the topic '%s', but the publisher is not activated",
      get_topic_name().c_str());
  }

protected:
  std::atomic<bool> is_activated_;
  ros::Publisher pub_;
};

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__PUBLISHER_HPP_
