#ifndef _romea_RosParam_hpp_
#define _romea_RosParam_hpp_

//ros
#include <ros/ros.h>

//romea
#include <romea_common/geodesy/GeodeticCoordinates.hpp>

//std
#include <exception>

namespace romea {

//-----------------------------------------------------------------------------
inline std::string searchParam(const ros::NodeHandle &nodeHandle,const std::string &paramName)
{
  std::string result;
  if(!nodeHandle.searchParam(paramName,result))
  {
    throw(std::runtime_error("Failed to find "+ paramName +" from param server"));
  }
  return result;
}

//-----------------------------------------------------------------------------
template <typename T>
inline T loadParam(const ros::NodeHandle &nodeHandle,
                   const std::string &paramName)
{
  T value;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,value))
  {
    throw(std::runtime_error("Failed to read "+ resolvedParamName +" from param server"));
  }
  return value;
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::vector<T> loadVector(const ros::NodeHandle &nodeHandle,
                                 const std::string &paramName)
{
  std::vector<T> vector;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,vector))
  {
    throw(std::runtime_error("Failed to read "+ resolvedParamName +" from param server"));
  }
  return vector;
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::map<std::string,T> loadMap(const ros::NodeHandle &nodeHandle,
                                       const std::string &paramName)
{

  XmlRpc::XmlRpcValue xml_rpc_map;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,xml_rpc_map))
  {
    throw(std::runtime_error("Failed to read "+ resolvedParamName +" from param server"));
  }

  //  xml_map.getType() TypeStruct

  std::map<std::string,T> map;
  XmlRpc::XmlRpcValue::ValueStruct::const_iterator it;
  for(it = xml_rpc_map.begin() ; it != xml_rpc_map.end(); ++it)
  {
    if(map.find(it->first)==map.end())
    {
      map.emplace(it->first,XmlRpc::XmlRpcValue(it->second));
    }
    else
    {
      throw(std::runtime_error("Key "+it->first+ "is already exist cannot load map "+resolvedParamName+ " from param server"));
    }
  }

  return map;
}


//-----------------------------------------------------------------------------
inline bool loadGeodeticCoordinates(const ros::NodeHandle &nodeHandle,
                                    const std::string &paramName,
                                    GeodeticCoordinates & coordinates)
{

  std::vector<double> vector;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,vector))
  {
    return false;
  }

  coordinates= GeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
  return true;
}

//-----------------------------------------------------------------------------
inline GeodeticCoordinates loadGeodeticCoordinates(const ros::NodeHandle &nodeHandle,
                                                   const std::string &paramName)
{
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  std::vector<double> vector = loadVector<double>(nodeHandle,resolvedParamName);
  return GeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
}

}

#endif

