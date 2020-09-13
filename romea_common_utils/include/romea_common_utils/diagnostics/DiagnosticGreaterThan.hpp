#ifndef _romea_DiagnosticGreaterThan_hpp_
#define _romea_DiagnosticGreaterThan_hpp_


//ros
#include <diagnostic_updater/diagnostic_updater.h>

//boost
#include <boost/optional.hpp>

//std
#include <mutex>

namespace romea {



template <typename T>
class  DiagnosticGreaterThan : public diagnostic_updater::DiagnosticTask
{

public:

  DiagnosticGreaterThan(const std::string &name);

  DiagnosticGreaterThan(const std::string &name,
                        const T &minimal_value);

  DiagnosticGreaterThan(const std::string &name,
                        const T &minimal_value,
                        const T &epsilon);

  void setEpsilonValue(const T & epsilon);

  void setMinimalValue(const T &minimal_value);

  const T & getMinimalValue() const;

  void update(const T & value);

  void reset();

  bool isOk() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private:

  boost::optional<T> value_;
  T minimal_value_;
  T epsilon_;
  mutable std::mutex mutex_;
};

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticGreaterThan<T>::DiagnosticGreaterThan(const std::string &name):
  DiagnosticTask(name),
  value_(),
  minimal_value_(),
  epsilon_(),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticGreaterThan<T>::DiagnosticGreaterThan(const std::string & name,
                                                const T & minimal_value):
  DiagnosticTask(name),
  value_(),
  minimal_value_(minimal_value),
  epsilon_(std::numeric_limits<T>::epsilon()),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
DiagnosticGreaterThan<T>::DiagnosticGreaterThan(const std::string & name,
                                                const T & minimal_value,
                                                const T & epsilon):
  DiagnosticTask(name),
  value_(),
  minimal_value_(minimal_value),
  epsilon_(epsilon),
  mutex_()
{

}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticGreaterThan<T>::setMinimalValue(const T & minimal_value)
{
  minimal_value_=minimal_value;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticGreaterThan<T>::setEpsilonValue(const T & epsilon)
{
  epsilon_ = epsilon;
}

//-----------------------------------------------------------------------------
template< typename T>
const T &DiagnosticGreaterThan<T>::getMinimalValue()const
{
  return minimal_value_;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticGreaterThan<T>::update(const T & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_=value;
}

//-----------------------------------------------------------------------------
template< typename T>
void DiagnosticGreaterThan<T>::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_.reset();
}

//-----------------------------------------------------------------------------
template< typename T>
bool DiagnosticGreaterThan<T>::isOk()const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!value_.is_initialized() )
  {
    return false;
  }

  if(*value_ > minimal_value_-epsilon_)
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
void DiagnosticGreaterThan<T>::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(!value_.is_initialized() )
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" unknown.");
    stat.add(getName(),"unknown");
  }
  else
  {  if(*value_ > minimal_value_-epsilon_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getName()+ " OK.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too low.");
    }
    stat.add(getName(), *value_);
  }
}


}// namespace


#endif



