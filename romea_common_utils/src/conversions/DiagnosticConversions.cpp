#include "romea_common_utils/conversions/DiagnosticConversions.hpp"

namespace
{

}

namespace romea
{

//-----------------------------------------------------------------------------
void toRosDiagnosticMsg(const std::string & diagnoctic_name,
                        const std::string & hardware_id,
                        const DiagnosticReport & report,
                        diagnostic_msgs::DiagnosticStatus & msg)
{
  msg.level=static_cast<int>(worseStatus(report.diagnostics));
  msg.hardware_id=hardware_id;
  msg.name = diagnoctic_name;

  for(const auto & diagnostic : report.diagnostics)
  {
    msg.message+=diagnostic.message+" ";
  }

  for(const auto & p : report.info)
  {
    diagnostic_msgs::KeyValue key;
    key.key=p.first;
    key.value=p.second;
    msg.values.push_back(key);
  }
}


}
