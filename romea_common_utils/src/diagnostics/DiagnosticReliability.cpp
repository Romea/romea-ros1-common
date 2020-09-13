#include "romea_common_utils/diagnostics/DiagnosticReliability.hpp"

namespace romea {

//-----------------------------------------------------------------------------
DiagnosticReliability::DiagnosticReliability(const std::string &name):
  DiagnosticTask(name),
  reliability_(-1),
  low_reliability_theshold_(1),
  high_reliability_theshold_(0)

{

}

//-----------------------------------------------------------------------------
DiagnosticReliability::DiagnosticReliability(const std::string &name,
                                             const double & low_reliability_theshold,
                                             const double & high_reliability_theshold):
  DiagnosticTask(name),
  reliability_(),
  low_reliability_theshold_(low_reliability_theshold),
  high_reliability_theshold_(high_reliability_theshold)
{

}

//-----------------------------------------------------------------------------
void DiagnosticReliability::setLowReliabilityTheshold(const double &low_reliability_theshold)
{
  low_reliability_theshold_ = low_reliability_theshold;
}

//-----------------------------------------------------------------------------
void DiagnosticReliability::setHighReliabilityTheshold(const double &high_reliability_theshold)
{
  high_reliability_theshold_= high_reliability_theshold;
}

//-----------------------------------------------------------------------------
const double & DiagnosticReliability::getLowReliabilityTheshold()
{
  return low_reliability_theshold_;
}

//-----------------------------------------------------------------------------
const double & DiagnosticReliability::getHighReliabilityTheshold()
{
  return high_reliability_theshold_;
}

//-----------------------------------------------------------------------------
void DiagnosticReliability::update(const double & reliability)
{
  reliability_=reliability;
}

//-----------------------------------------------------------------------------
void DiagnosticReliability::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if(reliability_ < low_reliability_theshold_ )
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Reliability is too low");
  }
  else if(reliability_ < high_reliability_theshold_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Reliability is uncertain ");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Reliability is high");
  }
  stat.add(getName(), reliability_);

}


}// namespace



