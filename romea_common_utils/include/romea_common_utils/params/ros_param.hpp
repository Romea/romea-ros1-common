#ifndef _romea_RosParam_hpp_
#define _romea_RosParam_hpp_

//ros
#include <ros/ros.h>

//romea
#include <romea_core_common/geodesy/GeodeticCoordinates.hpp>

//std
#include <exception>

namespace romea {

//-----------------------------------------------------------------------------
inline std::string search_param(const ros::NodeHandle &nodeHandle,const std::string &paramName)
{
  std::string result;
  if(!nodeHandle.searchParam(paramName,result))
  {
    std::stringstream msg;
    msg << "Failed to find ";
    msg << paramName;
    msg << " from param server";
    throw(std::runtime_error(msg.str()));
  }
  return result;
}

//-----------------------------------------------------------------------------
template <typename T>
inline T load_param(const ros::NodeHandle &nodeHandle,
                   const std::string &paramName)
{
  T value;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,value))
  {
    std::stringstream msg;
    msg << "Failed to load ";
    msg << resolvedParamName;
    msg << " from param server";
    throw(std::runtime_error(msg.str()));
  }
  return value;
}

//-----------------------------------------------------------------------------
template <typename T>
inline T load_param_or(const ros::NodeHandle &nodeHandle,
                       const std::string &paramName,
                       const T & default_value)
{
  return nodeHandle.param<T>(paramName,default_value);
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::vector<T> load_vector(const ros::NodeHandle &nodeHandle,
                                 const std::string &paramName)
{
  std::vector<T> vector;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,vector))
  {
    std::stringstream msg;
    msg << "Failed to read ";
    msg << resolvedParamName;
    msg << " from param server";
    throw(std::runtime_error(msg.str()));
  }
  return vector;
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::map<std::string,T> load_map(const ros::NodeHandle &nodeHandle,
                                       const std::string &paramName)
{

  XmlRpc::XmlRpcValue xml_rpc_map;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,xml_rpc_map))
  {
    std::stringstream msg;
    msg << "Failed to read ";
    msg << resolvedParamName;
    msg << " from param server";
    throw(std::runtime_error(msg.str()));
  }

  //  xml_map.getType() TypeStruct

  std::map<std::string,T> map;
  XmlRpc::XmlRpcValue::ValueStruct::const_iterator it;
  for(const auto & [key,value] : xml_rpc_map)
  {
    //c++20 replace by ! map.contains(key)
    if(map.find(key)==map.end())
    {
      map.emplace(key,XmlRpc::XmlRpcValue(value));
    }
    else
    {
      std::stringstream msg;
      msg << "Key ";
      msg << key;
      msg << "is already exist cannot load map";
      msg << resolvedParamName;
      msg << " from param server";
      throw(std::runtime_error(msg.str()));
    }
  }

  return map;
}


//-----------------------------------------------------------------------------
inline bool load_geodetic_coordinates(const ros::NodeHandle &nodeHandle,
                                      const std::string &paramName,
                                      core::GeodeticCoordinates & coordinates)
{

  std::vector<double> vector;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,vector))
  {
    return false;
  }

  coordinates = core::makeGeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
  return true;
}

//-----------------------------------------------------------------------------
inline core::GeodeticCoordinates load_geodetic_coordinates(const ros::NodeHandle &nodeHandle,
                                                     const std::string &paramName)
{
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  std::vector<double> vector = load_vector<double>(nodeHandle,resolvedParamName);
  return core::makeGeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
}

}

#endif

