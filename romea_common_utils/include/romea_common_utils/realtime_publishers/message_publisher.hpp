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

#ifndef _romea_RealtimeMessagePublisher_hpp_
#define _romea_RealtimeMessagePublisher_hpp_

//ros
#include <realtime_tools/realtime_publisher.h>

namespace romea {

template <class DataType, class MessageType>
class RealtimeMessagePublisher
{

private :

  using RTPublisher = realtime_tools::RealtimePublisher<MessageType> ;

public :

  RealtimeMessagePublisher();

  RealtimeMessagePublisher(ros::NodeHandle &nh,
                           const std::string & topic_name,
                          const size_t &queue_size);

public :

  void init(ros::NodeHandle &nh,
            const std::string & topic_name,
            const size_t &queue_size);

  void publish(const DataType & data);

protected :

  std::share_ptr<RTPublisher> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeMessagePublisher<DataType,MessageType>::MessagePublisher():
  pub_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeMessagePublisher<DataType,MessageType>::RealtimeMessagePublisher(ros::NodeHandle & nh,
                                                                         const std::string & topic_name,
                                                                         const size_t &queue_size):
  pub_(nullptr)
{
  init(nh,topic_name,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeMessagePublisher<DataType,MessageType>::init(ros::NodeHandle & nh,
                                                          const std::string & topic_name,
                                                          const size_t &queue_size)
{
  pub_=std::make_shared<RTPublisher>(nh,topic_name,queue_size)
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeMessagePublisher<DataType,MessageType>::publish(const DataType &data)
{
  if(pub_->trylock())
  {
    toRosMsg(data,pub_->msg_);
    realtime_pub->unlockAndPublish();
  }
}

}

#endif
