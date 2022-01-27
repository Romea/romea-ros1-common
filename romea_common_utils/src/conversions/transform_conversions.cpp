//romea
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/geometry_conversions.hpp"

//ros
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace romea {

//-----------------------------------------------------------------------------
void to_romea(const geometry_msgs::Transform & tranform_msg,
             Eigen::Affine3d &eigen_transform)
{
  eigen_transform.translation() =Eigen::Vector3d(tranform_msg.translation.x,
                                                 tranform_msg.translation.y,
                                                 tranform_msg.translation.z);

  Eigen::Quaterniond q(tranform_msg.rotation.w,
                       tranform_msg.rotation.x,
                       tranform_msg.rotation.y,
                       tranform_msg.rotation.z);

  eigen_transform.linear()=q.toRotationMatrix();
}

//-----------------------------------------------------------------------------
void toRosTransformMsg(const Eigen::Affine3d &eigen_transform,
                       geometry_msgs::Transform & tranform_msg)
{
  to_ros_msg(eigen_transform.translation(),tranform_msg.translation);
  to_ros_msg(eigen_transform.linear(),tranform_msg.rotation);
}

//-----------------------------------------------------------------------------
Eigen::Affine3d lookupTransformOnce(const std::string& target_frame,
                                    const std::string& source_frame,
                                    const ros::Time& time,
                                    const ros::Duration timeout)
{

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto tranform_msg =tf_buffer.lookupTransform(target_frame,
                                               source_frame,
                                               time,
                                               timeout);
  return tf2::transformToEigen(tranform_msg);
}

}
