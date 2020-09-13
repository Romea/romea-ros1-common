#ifndef _romea_RosEigenParam_hpp_
#define _romea_RosEigenParam_hpp_

//ros
#include "RosParam.hpp"

//romea
#include "romea_common/containers/Eigen/VectorOfEigenVector.hpp"

namespace romea {


//-----------------------------------------------------------------------------
template <typename EigenVector>
inline bool loadEigenVector(const ros::NodeHandle &nodeHandle,
                            const std::string &paramName,
                            EigenVector & eigenVector)
{
  std::vector<typename EigenVector::Scalar> vector;
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  if(!nodeHandle.getParam(resolvedParamName,vector))
  {
    return false;
  }

  eigenVector = EigenVector(vector.data());
  return true;
}


//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector loadEigenVector(const ros::NodeHandle &nodeHandle,
                                   const std::string &paramName)
{
  std::string resolvedParamName = nodeHandle.resolveName(paramName);
  return EigenVector(loadVector<typename EigenVector::Scalar>(nodeHandle,resolvedParamName).data());
}

////-----------------------------------------------------------------------------
//template <typename V>
//inline bool  loadVectorOfEigenVector(ros::NodeHandle &nodeHandle,
//                                const std::string &paramName,
//                                VectorOfEigenVector<V> & vector,
//                                std::string & errorMessage)
//{

//  XmlRpc::XmlRpcValue xml_vector;
//  std::string resolvedParamName = nodeHandle.resolveName(paramName);
//  if(!nodeHandle.getParam(resolvedParamName,xml_vector))
//  {
//    errorMessage = "Failed to read "+ resolvedParamName +" from param server";
//    return false;
//  }

//  vector.resize(xml_vector.size());
//  for(int n=0; n<xml_vector.size();n++)
//  {
//    ROS_ASSERT(xml_vector[n].getType() == XmlRpc::XmlRpcValue::TypeArray);
//    ROS_ASSERT(xml_vector[n].size() == Eigen::internal::traits<V>::RowsAtCompileTime);
//    for(int row = 0; row<Eigen::internal::traits<V>::RowsAtCompileTime; row++ )
//    {
//      vector[n][row]=typename V::Scalar(xml_vector[n][row]);
//    }
//  }

//  return true;
//}

////-----------------------------------------------------------------------------
//template <typename V>
//inline VectorOfEigenVector<V> loadVectorOfEigenVector(ros::NodeHandle &nodeHandle,
//                                                      const std::string &paramName)
//{

//  VectorOfEigenVector<V> vector;
//  std::string errorMessage;
//  if(!loadVectorOfEigenVector(nodeHandle,paramName,vector,errorMessage))
//  {
//    ROS_ERROR("%s",errorMessage.c_str());
//  }
//  return vector;
//}


}

#endif

