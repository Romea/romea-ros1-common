#ifndef _romea_GeometryConversions_hpp_
#define _romea_GeometryConversions_hpp_

//ros
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <romea_common_msgs/PoseAndTwist2DStamped.h>

//eigen
#include <Eigen/Geometry>

//romea



namespace romea
{

void toRomea(const geometry_msgs::Vector3 & position_msg,
             Eigen::Vector3d & eigen_position);

void toRosMsg(const Eigen::Vector3d & eigen_position,
              geometry_msgs::Vector3 & position_msg);

void toRomea(const geometry_msgs::Quaternion & quaternion_msg,
             Eigen::Quaterniond & eigen_quaternion);

void toRosMsg(const Eigen::Quaterniond & eigen_quaternion,
              geometry_msgs::Quaternion &quaternion_msg);

void toRomea(const geometry_msgs::Quaternion & quaternion_msg,
             Eigen::Matrix3d & eigen_rotation_matrix);

void toRosMsg(const Eigen::Matrix3d & eigen_rotation_matrix,
              geometry_msgs::Quaternion &quaternion_msg);


void toRomea(const romea_common_msgs::Pose2D & position_msg,
             Eigen::Vector3d & eigen_position);


}

#endif
