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

#ifndef _romea_MessagePublisher_hpp_
#define _romea_MessagePublisher_hpp_

//ros
#include <ros/ros.h>

namespace romea {

template <class DataType, class MessageType>
class MessagePublisher
{

public :

  MessagePublisher();

  MessagePublisher(ros::NodeHandle &nh,
                   const std::string & topic_name,
                   const size_t &queue_size);

public :

  void init(ros::NodeHandle &nh,
                    const std::string & topic_name,
                    const size_t &queue_size);

  void publish(const DataType & data);

protected :

  ros::Publisher pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
MessagePublisher<DataType,MessageType>::MessagePublisher():
  pub_()
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
MessagePublisher<DataType,MessageType>::MessagePublisher(ros::NodeHandle & nh,
                                                         const std::string & topic_name,
                                                         const size_t &queue_size):
  pub_()
{
  init(nh,topic_name,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void MessagePublisher<DataType,MessageType>::init(ros::NodeHandle & nh,
                                                  const std::string & topic_name,
                                                  const size_t &queue_size)
{
  pub_=nh.advertise<MessageType>(topic_name,queue_size);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void MessagePublisher<DataType,MessageType>::publish(const DataType &data)
{
  boost::shared_ptr<MessageType> msg(new MessageType());
  to_ros_msg(data,*msg.get());
  pub_.publish(msg);
}

}

#endif
