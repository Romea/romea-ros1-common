#include "romea_common_utils/diagnostics/DiagnosticNLSE.hpp"

namespace romea {


//-----------------------------------------------------------------------------
DiagnosticNLSE::DiagnosticNLSE(const std::string &name):
  DiagnosticNLSE(name,0,-1)
{

}

//-----------------------------------------------------------------------------
DiagnosticNLSE::DiagnosticNLSE(const std::string &name,
                               const size_t & maximal_number_of_iterations,
                               const double & maximal_root_mean_square_error):
  DiagnosticTask(name),
  number_of_iterations_(0),
  maximal_number_of_iterations_(maximal_number_of_iterations),
  root_mean_square_error_(-1),
  maximal_root_mean_square_error_(maximal_root_mean_square_error),
  mutex_()
{

}

//-----------------------------------------------------------------------------
void DiagnosticNLSE::setMaximalNumberOfIterations(const size_t &maximal_number_of_iterations)
{
  maximal_number_of_iterations_ = maximal_number_of_iterations;
}

//-----------------------------------------------------------------------------
void DiagnosticNLSE::setMaximalRootMeanSquareError(const double &maximal_root_mean_square_error)
{
  maximal_root_mean_square_error_=maximal_root_mean_square_error;
}

//-----------------------------------------------------------------------------
const size_t & DiagnosticNLSE::getMaximalNumberOfIterations()const
{
  return maximal_number_of_iterations_;
}

//-----------------------------------------------------------------------------
const double & DiagnosticNLSE::getMaximalRootMeanSquareError() const
{
  return maximal_root_mean_square_error_;
}

//-----------------------------------------------------------------------------
void DiagnosticNLSE::update(const size_t & number_of_iterations,
                            const double & root_mean_square_error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  number_of_iterations_=number_of_iterations;
  root_mean_square_error_=root_mean_square_error;
}

//-----------------------------------------------------------------------------
void DiagnosticNLSE::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

  std::lock_guard<std::mutex> lock(mutex_);

  if(number_of_iterations_==0 || number_of_iterations_ == maximal_number_of_iterations_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName() +" init failed");
  }
  else
  {
    if(root_mean_square_error_ > maximal_root_mean_square_error_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName() +" did not converge");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getName() +" OK");
    }
  }

  stat.add(getName()+"_nb_iterations",number_of_iterations_);
  stat.add(getName()+"_rmse",root_mean_square_error_);

}

//-----------------------------------------------------------------------------
bool DiagnosticNLSE::isOk()const
{
  std::lock_guard<std::mutex> lock(mutex_);

  return number_of_iterations_!=0 &&
      number_of_iterations_ != maximal_number_of_iterations_ &&
      root_mean_square_error_ < maximal_root_mean_square_error_;
}

}
