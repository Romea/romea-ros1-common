#include "romea_common_utils/conversions/diagnostic_conversions.hpp"

namespace
{

}

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_diagnostic_msg(const std::string & diagnoctic_name,
                           const std::string & hardware_id,
                           const DiagnosticReport & report,
                           diagnostic_msgs::DiagnosticStatus & msg)
{
  msg.level=static_cast<int>(worseStatus(report.diagnostics));
  msg.hardware_id=hardware_id;
  msg.name = diagnoctic_name;

  for(const auto & diagnostic : report.diagnostics)
  {
    if(diagnostic.status!=DiagnosticStatus::OK)
    {
      msg.message+=diagnostic.message+" ";
    }
  }

  for(const auto & [info_name,info_status] : report.info)
  {
    diagnostic_msgs::KeyValue key;
    key.key=info_name;
    key.value=info_status;
    msg.values.push_back(key);
  }
}


}
