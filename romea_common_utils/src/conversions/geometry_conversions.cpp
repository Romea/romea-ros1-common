#include "romea_common_utils/conversions/geometry_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
void toRomea(const geometry_msgs::Vector3 &position_msg,
             Eigen::Vector3d & eigen_position)
{
  eigen_position.x()=position_msg.x;
  eigen_position.y()=position_msg.y;
  eigen_position.z()=position_msg.z;
}

//-----------------------------------------------------------------------------
void toRosMsg(const Eigen::Vector3d & eigen_position,
              geometry_msgs::Vector3 &position_msg)
{
  position_msg.x=eigen_position.x();
  position_msg.y=eigen_position.y();
  position_msg.z=eigen_position.z();

}

//-----------------------------------------------------------------------------
void toRomea(const geometry_msgs::Quaternion & quaternion_msg,
             Eigen::Quaterniond & eigen_quaternion)
{
  eigen_quaternion.x()=quaternion_msg.x;
  eigen_quaternion.y()=quaternion_msg.y;
  eigen_quaternion.z()=quaternion_msg.z;
  eigen_quaternion.w()=quaternion_msg.w;
}

//-----------------------------------------------------------------------------
void toRosMsg(const Eigen::Quaterniond & eigen_quaternion,
              geometry_msgs::Quaternion & quaternion_msg)
{
  quaternion_msg.x=eigen_quaternion.x();
  quaternion_msg.y=eigen_quaternion.y();
  quaternion_msg.z=eigen_quaternion.z();
  quaternion_msg.w=eigen_quaternion.w();
}


//-----------------------------------------------------------------------------
void toRomea(const geometry_msgs::Quaternion & quaternion_msg,
             Eigen::Matrix3d & eigen_rotation_matrix)
{
  Eigen::Quaterniond quaternion;
  toRomea(quaternion_msg,quaternion);
  eigen_rotation_matrix=quaternion.toRotationMatrix();
}

//-----------------------------------------------------------------------------
void toRosMsg(const Eigen::Matrix3d & eigen_rotation_matrix,
              geometry_msgs::Quaternion & quaternion_msg)
{
  Eigen::Quaterniond quaternion(eigen_rotation_matrix);
  toRosMsg(quaternion,quaternion_msg);
}

}
