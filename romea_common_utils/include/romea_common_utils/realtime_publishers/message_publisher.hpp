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
