#ifndef _romea_ReliabilityDiagnostic_hpp_
#define _romea_ReliabilityDiagnostic_hpp_

//ros
#include <diagnostic_updater/diagnostic_updater.h>

namespace romea {

class  DiagnosticReliability : public diagnostic_updater::DiagnosticTask
{

public:

  DiagnosticReliability(const std::string &name);

  DiagnosticReliability(const std::string &name,
                        const double & low_reliability_threshold,
                        const double & high_reliability_threshold);

  void setLowReliabilityTheshold(const double &bad_reliability_theshold);

  void setHighReliabilityTheshold(const double &good_reliability_theshold);

  const double & getLowReliabilityTheshold();

  const double & getHighReliabilityTheshold();

  void update(const double & reliability);

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private:

  double reliability_;
  double low_reliability_theshold_;
  double high_reliability_theshold_;

};


}// namespace


#endif
