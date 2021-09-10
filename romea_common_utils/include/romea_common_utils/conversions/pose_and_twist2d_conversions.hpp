#ifndef _romea_PoseAndTwist2DConversions_hpp_
#define _romea_PoseAndTwist2DConversions_hpp_

//romea
#include <romea_common/geometry/PoseAndTwist2D.hpp>
#include "pose2d_conversions.hpp"
#include "twist2d_conversions.hpp"

//romea_ros_msg
#include <romea_common_msgs/PoseAndTwist2D.h>
#include <romea_common_msgs/PoseAndTwist2DStamped.h>

//ros
#include <tf/tf.h>

namespace romea
{


void toRosMsg(const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void toRosMsg(const ros::Time & stamp,
              const std::string & frame_id,
              const PoseAndTwist2D & romea_pose_and_twist2d,
              romea_common_msgs::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped);

PoseAndTwist2D toRomea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg);

void toRomea(const romea_common_msgs::PoseAndTwist2D &ros_pose_and_twist2d_msg,
             PoseAndTwist2D & romea_pose_and_twist2d);


}

#endif
