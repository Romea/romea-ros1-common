#ifndef _romea_DiagnosticConversions_hpp_
#define _romea_DiagnosticConversions_hpp_

//romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

//ros
#include <diagnostic_msgs/DiagnosticArray.h>

namespace romea
{
void to_ros_diagnostic_msg(const std::string & diagnostic_name,
                           const std::string & hardware_id,
                           const core::DiagnosticReport & report,
                           diagnostic_msgs::DiagnosticStatus &msg);

}

#endif
