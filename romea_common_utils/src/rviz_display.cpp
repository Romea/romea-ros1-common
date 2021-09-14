#include "romea_common_utils/rviz_display.hpp"
#include <tf2_eigen/tf2_eigen.h>

namespace {

//-----------------------------------------------------------------------------
inline  Eigen::Isometry3d computeRobotPose(const romea::Pose2D & bodyPose2D,
                                           const double & positionAlongZBodyAxis)
{
  return Eigen::Translation3d(bodyPose2D.position.x(),
                              bodyPose2D.position.y(),
                              positionAlongZBodyAxis)*
      Eigen::AngleAxisd(bodyPose2D.yaw,Eigen::Vector3d::UnitZ());

}

//-----------------------------------------------------------------------------
inline  Eigen::Isometry3d computeEllipsePose(const romea::Ellipse & ellipse,
                                             const double & positionAlongZBodyAxis)
{
  return Eigen::Translation3d(ellipse.getCenterPosition().x(),
                              ellipse.getCenterPosition().y(),
                              positionAlongZBodyAxis)*
      Eigen::AngleAxisd(ellipse.getOrientation(), Eigen::Vector3d::UnitZ());
}

}

namespace romea {


//-----------------------------------------------------------------------------
void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const Pose2D & bodyPose2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis,
             double scaleAlongZBodyAxis,
             double sigma)
{

  Ellipse ellipse= uncertaintyEllipse(bodyPose2D,sigma);

  geometry_msgs::Vector3 scale;
  scale.x = ellipse.getMajorRadius();
  scale.y = ellipse.getMinorRadius();
  scale.z = scaleAlongZBodyAxis;

  geometry_msgs::Pose ellipse_pose_msg =
      tf2::toMsg(computeEllipsePose(ellipse,positionAlongZBodyAxis));
  rvizVisualTool.publishSphere(ellipse_pose_msg,
                               rvizVisualTool.getColor(color),
                               scale);

  geometry_msgs::Pose robot_pose_msg=
      tf2::toMsg(computeRobotPose(bodyPose2D,positionAlongZBodyAxis));
  rvizVisualTool.publishAxis(robot_pose_msg,1);

}

//-----------------------------------------------------------------------------
void publish(rviz_visual_tools::RvizVisualTools & rvizVisualTool,
             const Position2D & bodyPosition2D,
             const rviz_visual_tools::colors & color,
             double positionAlongZBodyAxis,
             double scaleAlongZBodyAxis,
             double sigma)
{

  Ellipse ellipse= uncertaintyEllipse(bodyPosition2D,sigma);

  geometry_msgs::Vector3 scale;
  scale.x = ellipse.getMajorRadius();
  scale.y = ellipse.getMinorRadius();
  scale.z = scaleAlongZBodyAxis;

  geometry_msgs::Pose ellipse_pose_msg=
      tf2::toMsg(computeEllipsePose(ellipse,positionAlongZBodyAxis));
  rvizVisualTool.publishSphere(ellipse_pose_msg,
                               rvizVisualTool.getColor(color),scale);

}




}