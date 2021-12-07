#ifndef _romea_Pose3DConversions_hpp_
#define _romea_Pose3DConversions_hpp_

//std
#include <string>

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose3D.hpp>

//ros
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace romea
{


void toRosTransformMsg(const Pose3D & romea_pose_3d,
                       geometry_msgs::Transform & ros_transform_msg);


void toRosTransformMsg(const ros::Time &stamp,
                       const Pose3D &romea_pose_3d,
                       const std::string &frame_id,
                       const std::string &child_frame_id,
                       geometry_msgs::TransformStamped &tf_msg);

void toRosMsg(const Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovariance & ros_pose_msg);

void toRosMsg(const ros::Time & stamp,
              const std::string &frame_id,
              const Pose3D & romea_pose_3d,
              geometry_msgs::PoseWithCovarianceStamped & ros_pose_msg);


void toRomea(const geometry_msgs::PoseWithCovariance & ros_pose_msg,
             Pose3D & romea_pose_3d);

Pose3D toRomea(const geometry_msgs::PoseWithCovariance & ros_pose_msg);

}

#endif
