#ifndef _romea_OdomPublisher_hpp_
#define _romea_OdomPublisher_hpp_

//ros
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>

//romea
#include "../conversions/time_conversions.hpp"

namespace romea
{

template <class DataType>
class OdomPublisher
{

public :

  OdomPublisher();

  OdomPublisher(ros::NodeHandle &nh,
                const std::string & topic_name,
                const std::string & frame_id,
                const std::string & child_frame_id,
                const size_t &queue_size);
public :

  void init(ros::NodeHandle &nh,
            const std::string & topic_name,
            const std::string & frame_id,
            const std::string & child_frame_id,
            const size_t &queue_size);

  void publish(const core::Duration & duration,
               const DataType &data);

  void publish(const ros::Time & stamp,
               const DataType &data);

public :

  std::string frame_id_;
  std::string child_frame_id_;
  ros::Publisher pub_;
};

//-----------------------------------------------------------------------------
template <class DataType>
OdomPublisher<DataType>::OdomPublisher():
  frame_id_(),
  child_frame_id_(),
  pub_()
{

}

//-----------------------------------------------------------------------------
template <class DataType>
OdomPublisher<DataType>::OdomPublisher(ros::NodeHandle &nh,
                                       const std::string & topic_name,
                                       const std::string & frame_id,
                                       const std::string & child_frame_id,
                                       const size_t &queue_size):
  frame_id_(),
  child_frame_id_(),
  pub_()
{
  init(nh,topic_name,frame_id,child_frame_id,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType>
void OdomPublisher<DataType>::init(ros::NodeHandle &nh,
                                   const std::string & topic_name,
                                   const std::string & frame_id,
                                   const std::string & child_frame_id,
                                   const size_t &queue_size)
{
  assert(!frame_id.empty());
  assert(!child_frame_id.empty());

  child_frame_id_ = child_frame_id;
  frame_id_=frame_id;
  pub_=nh.advertise<nav_msgs::Odometry>(topic_name,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType>
void OdomPublisher<DataType>::publish(const core::Duration & duration,
                                      const DataType &data)
{
  publish(to_ros_time(duration),data);
}

//-----------------------------------------------------------------------------
template <class DataType>
void OdomPublisher<DataType>::publish(const ros::Time & stamp,
                                      const DataType &data)
{
  boost::shared_ptr<nav_msgs::Odometry> msg(new nav_msgs::Odometry());
  to_ros_odom_msg(stamp,data,frame_id_,child_frame_id_,*msg.get());
  pub_.publish(msg);
}

}


#endif
