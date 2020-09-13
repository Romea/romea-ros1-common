//ros
#include <ros/ros.h>
#include "../conversions/TimeConversions.hpp"

namespace romea {

template <class DataType, class MessageType>
class StampedMessagePublisher
{

public :

  StampedMessagePublisher();

  StampedMessagePublisher(ros::NodeHandle &nh,
                          const std::string & topic_name,
                          const std::string & frame_id,
                          const size_t &queue_size);

  virtual ~StampedMessagePublisher()=default;

public :

  virtual void init(ros::NodeHandle &nh,
                    const std::string & topic_name,
                    const std::string & frame_id,
                    const size_t &queue_size);

  virtual void publish(const ros::Time & stamp,
                       const DataType & data);


  virtual void publish(const romea::Duration & duration,
                       const DataType & data);

protected :

  std::string frame_id_;
  ros::Publisher pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
StampedMessagePublisher<DataType,MessageType>::StampedMessagePublisher():
  frame_id_(),
  pub_()
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
StampedMessagePublisher<DataType,MessageType>::StampedMessagePublisher(ros::NodeHandle & nh,
                                                                       const std::string & topic_name,
                                                                       const std::string & frame_id,
                                                                       const size_t &queue_size):
  frame_id_(),
  pub_()
{
  init(nh,topic_name,frame_id,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::init(ros::NodeHandle & nh,
                                                         const std::string & topic_name,
                                                         const std::string & frame_id,
                                                         const size_t &queue_size)
{
  assert(!frame_id.empty());
  frame_id_=frame_id;
  pub_=nh.advertise<MessageType>(topic_name,queue_size);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::publish(const ros::Time & stamp,
                                                            const DataType &data)
{
  boost::shared_ptr<MessageType> msg(new MessageType());
  toRosMsg(stamp,frame_id_,data,*msg.get());
  pub_.publish(msg);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::publish(const romea::Duration & duration,
                                                            const DataType & data)
{
  publish(toROSTime(duration),data);
}

}
