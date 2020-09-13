//ros
#include <ros/ros.h>
#include "../conversions/TimeConversions.hpp"

namespace romea {

template <class DataType, class MessageType>
class MessagePublisher
{

public :

  MessagePublisher();

  MessagePublisher(ros::NodeHandle &nh,
                   const std::string & topic_name,
                   const size_t &queue_size);

  virtual ~MessagePublisher()=default;

public :

  virtual void init(ros::NodeHandle &nh,
                    const std::string & topic_name,
                    const size_t &queue_size);

  virtual void publish(const DataType & data);

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
  toRosMsg(data,*msg.get());
  pub_.publish(msg);
}

}
