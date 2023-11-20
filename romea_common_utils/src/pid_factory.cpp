#include "romea_common_utils/pid_factory.hpp"
#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{

std::unique_ptr<PID> make_pid(ros::NodeHandle &nh)
{
  return std::make_unique<PID>(load_param<double>(nh,"kp"),
                               load_param<double>(nh,"ki"),
                               load_param<double>(nh,"kd"),
                               load_param<double>(nh,"imin"),
                               load_param<double>(nh,"imax"),
                               load_param<double>(nh,"error_epsilon"));
}

}

