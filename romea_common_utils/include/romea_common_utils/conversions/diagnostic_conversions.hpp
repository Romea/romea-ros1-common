#ifndef _romea_DiagnosticConversions_hpp_
#define _romea_DiagnosticConversions_hpp_

#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace romea
{
void toRosDiagnosticMsg(const std::string & diagnostic_name,
                        const std::string & hardware_id,
                        const DiagnosticReport & report,
                        diagnostic_msgs::DiagnosticStatus &msg);

}

#endif
