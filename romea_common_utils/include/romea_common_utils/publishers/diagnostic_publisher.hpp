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

#ifndef _romea_DiagnosticPublisher_hpp_
#define _romea_DiagnosticPublisher_hpp_

//ros
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

//romea
#include "../conversions/time_conversions.hpp"
#include "../conversions/diagnostic_conversions.hpp"

namespace romea
{

template <class DataType>
class DiagnosticPublisher
{

public :

  DiagnosticPublisher();

  DiagnosticPublisher(ros::NodeHandle &nh,
                      const std::string & diagnostic_name,
                      const double & diagnostic_period,
                      const std::string & hardware_id_="",
                      const std::string & topic_name="/diagnostics");

public :

  void init(ros::NodeHandle &nh,
            const std::string & diagnostic_name,
            const double & diagnostic_period,
            const std::string & hardware_id_="",
            const std::string & topic_name="/diagnostics");

  void publish(const core::Duration & duration,
               const DataType &data);

  void publish(const ros::Time & stamp,
               const DataType &data);

private :

  void publish_(const ros::Time & stamp,
                const DataType &data);

private :

  std::string diagnostic_name_;
  std::string hardware_id_;
  ros::Publisher pub_;
  ros::Time next_time_;
  double diagnostic_period_;
};

//-----------------------------------------------------------------------------
template <class DataType>
DiagnosticPublisher<DataType>::DiagnosticPublisher():
  diagnostic_name_(),
  hardware_id_(),
  pub_(),
  next_time_(),
  diagnostic_period_()
{

}

//-----------------------------------------------------------------------------
template <class DataType>
DiagnosticPublisher<DataType>::DiagnosticPublisher(ros::NodeHandle &nh,
                                                   const std::string & diagnostic_name,
                                                   const double & diagnostic_period,
                                                   const std::string & hardware_id,
                                                   const std::string & topic_name):
  diagnostic_name_(),
  hardware_id_(),
  pub_(),
  next_time_(),
  diagnostic_period_(0)
{
  init(nh,diagnostic_name,diagnostic_period,hardware_id,topic_name);
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::init(ros::NodeHandle &nh,
                                         const std::string & diagnostic_name,
                                         const double & diagnostic_period,
                                         const std::string & hardware_id,
                                         const std::string & topic_name)
{
  diagnostic_name_ = diagnostic_name;
  hardware_id_ = hardware_id;
  diagnostic_period_=diagnostic_period;
  pub_=nh.advertise<diagnostic_msgs::DiagnosticArray>(topic_name,1);
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish(const core::Duration & duration,
                                            const DataType &data)
{
  publish(to_ros_time(duration),data);

}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish(const ros::Time & stamp,
                                            const DataType & data)
{
  if(stamp>next_time_)
  {
    publish_(stamp,data);
    next_time_ = stamp+ ros::Duration(diagnostic_period_);
  }
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish_(const ros::Time & stamp,
                                             const DataType & data)
{
  using namespace diagnostic_msgs;
  boost::shared_ptr<DiagnosticArray> msg(new DiagnosticArray());
  msg->header.stamp=stamp;
  msg->status.push_back(diagnostic_msgs::DiagnosticStatus());
  to_ros_diagnostic_msg(diagnostic_name_,hardware_id_,data,msg->status.back());
  pub_.publish(msg);
}

}


#endif
