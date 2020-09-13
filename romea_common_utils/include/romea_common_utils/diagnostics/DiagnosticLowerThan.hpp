#ifndef _romea_DiagnosticLowerThan_hpp_
#define _romea_DiagnosticLowerThan_hpp_


//ros
#include <diagnostic_updater/diagnostic_updater.h>

//boost
#include <boost/optional.hpp>

namespace romea {



template <typename T>
class  DiagnosticLowerThan : public diagnostic_updater::DiagnosticTask
{

public:

  DiagnosticLowerThan(const std::string &name);

  DiagnosticLowerThan(const std::string &name,
                      const T &maximal_value);

  DiagnosticLowerThan(const std::string &name,
                      const T &maximal_value,
                      const T &epsilon);

  void setEpsilonValue(const T & epsilon);

  void setMaximalValue(const T &maximal_value);

  const T & getMaximalValue() const;

  void update(const T & value);

  void reset();

  bool isOk() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private:

  boost::optional<T> value_;
  T maximal_value_;
  T epsilon_;

  mutable std::mutex mutex_;
};

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticLowerThan<T>::DiagnosticLowerThan(const std::string &name):
  DiagnosticTask(name),
  value_(),
  maximal_value_(),
  epsilon_(),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticLowerThan<T>::DiagnosticLowerThan(const std::string & name,
                                            const T & maximal_value):
  DiagnosticTask(name),
  value_(),
  maximal_value_(maximal_value),
  epsilon_(std::numeric_limits<T>::epsilon()),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticLowerThan<T>::DiagnosticLowerThan(const std::string & name,
                                            const T & maximal_value,
                                            const T & epsilon):
  DiagnosticTask(name),
  value_(),
  maximal_value_(maximal_value),
  epsilon_(epsilon),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticLowerThan<T>::setMaximalValue(const T & maximal_value)
{
  maximal_value_=maximal_value;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticLowerThan<T>::setEpsilonValue(const T & epsilon)
{
  epsilon_ = epsilon;
}

//-----------------------------------------------------------------------------
template< typename T>
const T &DiagnosticLowerThan<T>::getMaximalValue()const
{
  return maximal_value_;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticLowerThan<T>::update(const T & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_=value;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticLowerThan<T>::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_.reset();
}

//-----------------------------------------------------------------------------
template< typename T>
bool DiagnosticLowerThan<T>::isOk()const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!value_.is_initialized() )
  {
    return false;
  }

  if(*value_ < maximal_value_+epsilon_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticLowerThan<T>::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(!value_.is_initialized() )
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" unknown.");
    stat.add(getName(),"unknown");
  }
  else
  {
    if(*value_ < maximal_value_+epsilon_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getName()+ " OK.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too high.");
    }
    stat.add(getName(), *value_);
  }
}


}// namespace


#endif



