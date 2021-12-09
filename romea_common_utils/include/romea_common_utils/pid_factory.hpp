#ifndef _romea_pid_factory_hpp_
#define _romea_pid_factory_hpp_

//ros
#include <ros/ros.h>

//romea
#include "romea_core_common/control/PID.hpp"

namespace romea
{

std::unique_ptr<PID> make_pid(ros::NodeHandle &nh);

}

#endif
