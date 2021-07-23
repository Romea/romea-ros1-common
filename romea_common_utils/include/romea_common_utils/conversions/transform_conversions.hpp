#ifndef _romea_TransformConversions_hpp_
#define _romea_TransformConversions_hpp_

//ros
#include <geometry_msgs/Transform.h>


//eigen
#include <Eigen/Geometry>

namespace romea {

void toRomea(const geometry_msgs::Transform & tranform_msg,
             Eigen::Affine3d &eigen_transform);

void toRosTransformMsg(const Eigen::Affine3d &eigen_transform,
                       geometry_msgs::Transform & tranform_msg);

Eigen::Affine3d lookupTransformOnce(const std::string& target_frame,
                                    const std::string& source_frame,
                                    const ros::Time& time,
                                    const ros::Duration timeout);

}

#endif
