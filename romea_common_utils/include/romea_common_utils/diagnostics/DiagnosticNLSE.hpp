#ifndef _romea_DiagnosticNLSE_hpp_
#define _romea_DiagnosticNLSE_hpp_

//ros
#include <diagnostic_updater/diagnostic_updater.h>

//std
#include <mutex>

namespace romea {

class DiagnosticNLSE : public diagnostic_updater::DiagnosticTask
{

public :

  DiagnosticNLSE(const std::string &name);

  DiagnosticNLSE(const std::string &name,
                 const size_t & maximal_number_of_iterations,
                 const double & maximal_root_mean_square_error);

  void setMaximalNumberOfIterations(const size_t &maximal_number_of_iterations_);

  void setMaximalRootMeanSquareError(const double &maximal_root_mean_square_error);

  const size_t & getMaximalNumberOfIterations()const;

  const double & getMaximalRootMeanSquareError()const;

  void update(const size_t & number_of_iterations,const double & root_mean_square_error);

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

  bool isOk()const;

private:

  size_t number_of_iterations_;
  size_t maximal_number_of_iterations_;
  double root_mean_square_error_;
  double maximal_root_mean_square_error_;
  mutable std::mutex mutex_;
};

}
#endif
