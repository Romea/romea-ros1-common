#ifndef _romea_DiagnosticEqualTo_hpp_
#define _romea_DiagnosticEqualTo_hpp_

//ros
#include <diagnostic_updater/diagnostic_updater.h>

//boost
#include <boost/optional.hpp>

namespace romea {


template <typename T>
class  DiagnosticEqualTo : public diagnostic_updater::DiagnosticTask
{

public:

  DiagnosticEqualTo(const std::string &name);

  DiagnosticEqualTo(const std::string &name,
                    const T & desired_value);

  DiagnosticEqualTo(const std::string &name,
                    const T & desired_value,
                    const T & epsilon);

  void setEpsilonValue(const T & epsilon);

  void setDesiredValue(const T & desired_value);

  const double & getDesiredValue()const;

  void update(const T & value);

  void reset();

  bool isOk() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private:

  boost::optional<T> value_;
  T desired_value_;
  T epsilon_;
  mutable std::mutex mutex_;
};


//-----------------------------------------------------------------------------
template <typename T>
DiagnosticEqualTo<T>::DiagnosticEqualTo(const std::string & name):
  DiagnosticTask(name),
  value_(),
  desired_value_(0),
  epsilon_(std::numeric_limits<T>::epsilon()),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template <typename T>
DiagnosticEqualTo<T>::DiagnosticEqualTo(const std::string & name,
                                        const T &desired_value):
  DiagnosticTask(name),
  value_(),
  desired_value_(desired_value),
  epsilon_(std::numeric_limits<T>::epsilon()),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template <typename T>
DiagnosticEqualTo<T>::DiagnosticEqualTo(const std::string & name,
                                        const T &desired_value,
                                        const T &epsilon):
  DiagnosticTask(name),
  value_(),
  desired_value_(desired_value),
  epsilon_(epsilon)
{

}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticEqualTo<T>::setEpsilonValue(const T & epsilon)
{
  epsilon_ = epsilon;
}

//-----------------------------------------------------------------------------
template <typename T>
void DiagnosticEqualTo<T>::setDesiredValue(const T &desired_value)
{
  desired_value_=desired_value;
}

//-----------------------------------------------------------------------------
template <typename T>
const double & DiagnosticEqualTo<T>::getDesiredValue()const
{
  return desired_value_;
}

//-----------------------------------------------------------------------------
template <typename T>
void DiagnosticEqualTo<T>::update(const T & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_=value;
}

//-----------------------------------------------------------------------------
template <typename T>
void DiagnosticEqualTo<T>::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_.reset();
}


//-----------------------------------------------------------------------------
template <typename T>
bool DiagnosticEqualTo<T>::isOk() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!value_.is_initialized() )
  {
    return false;
  }

  if(*value_ > desired_value_-epsilon_ &&
     *value_ < desired_value_+epsilon_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------------------------------------------------
template <typename T>
void DiagnosticEqualTo<T>::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

  std::lock_guard<std::mutex> lock(mutex_);

  if(value_.is_initialized())
  {

    if(*value_ < desired_value_-epsilon_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too low.");
    }
    else if(*value_ > desired_value_+epsilon_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too high.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getName()+" OK.");
    }

    stat.add(getName(), *value_);

  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" unknown.");
    stat.add(getName(), "Unkown");
  }
}


}

#endif
