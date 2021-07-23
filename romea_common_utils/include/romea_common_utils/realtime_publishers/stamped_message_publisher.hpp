#ifndef _romea_RealtimeStampedMessagePublisher_hpp_
#define _romea_RealtimeStampedMessagePublisher_hpp_

//ros
#include "../conversions/time_conversions.hpp"
#include <realtime_tools/realtime_publisher.h>

namespace romea {

template <class DataType, class MessageType>
class RealtimeStampedMessagePublisher
{

private :

  using RTPublisher =  typename realtime_tools::RealtimePublisher<MessageType> ;

public :

  RealtimeStampedMessagePublisher();

  RealtimeStampedMessagePublisher(ros::NodeHandle &nh,
                                  const std::string & topic_name,
                                  const std::string & frame_id,
                                  const size_t &queue_size);
public :

  void init(ros::NodeHandle &nh,
            const std::string & topic_name,
            const std::string & frame_id,
            const size_t &queue_size);

  void publish(const ros::Time & stamp,
               const DataType & data);


  void publish(const romea::Duration & duration,
               const DataType & data);

protected :

  std::string frame_id_;
  std::shared_ptr<RTPublisher> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeStampedMessagePublisher<DataType,MessageType>::RealtimeStampedMessagePublisher():
  frame_id_(),
  pub_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
RealtimeStampedMessagePublisher<DataType,MessageType>::RealtimeStampedMessagePublisher(ros::NodeHandle & nh,
                                                                                       const std::string & topic_name,
                                                                                       const std::string & frame_id,
                                                                                       const size_t &queue_size):
  frame_id_(),
  pub_(nullptr)
{
  init(nh,topic_name,frame_id,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::init(ros::NodeHandle & nh,
                                                                 const std::string & topic_name,
                                                                 const std::string & frame_id,
                                                                 const size_t &queue_size)
{
  assert(!frame_id.empty());
  frame_id_ = frame_id;
  pub_= std::make_shared<RTPublisher>(nh,topic_name,queue_size);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::publish(const ros::Time & stamp,
                                                                    const DataType &data)
{
  if(pub_->trylock())
  {
      toRosMsg(stamp,frame_id_,data,pub_->msg_);
      pub_->unlockAndPublish();
  }
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void RealtimeStampedMessagePublisher<DataType,MessageType>::publish(const romea::Duration & duration,
                                                                    const DataType & data)
{
  publish(toROSTime(duration),data);
}

}

#endif
