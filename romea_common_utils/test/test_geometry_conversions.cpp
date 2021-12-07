// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include <romea_core_common/math/EulerAngles.hpp>
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include "romea_common_utils/conversions/pose3d_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_common_utils/conversions/twist3d_Conversions.hpp"

//-----------------------------------------------------------------------------
class TestTwist2DConversion : public ::testing::Test
{
public :

  TestTwist2DConversion():
    stamp(1000),
    frame_id("foo"),
    romea_twist2d(),
    ros_twist2d_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_twist2d.linearSpeeds.x()=1;
    romea_twist2d.linearSpeeds.y()=2;
    romea_twist2d.angularSpeed=3;
    fillEigenCovariance(romea_twist2d.covariance);
    romea::toRosMsg(stamp,frame_id,romea_twist2d,ros_twist2d_msg);
  }

  ros::Time stamp;
  std::string frame_id;
  romea::Twist2D romea_twist2d;
  romea_localisation_msgs::Twist2DStamped ros_twist2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRomeatoRosMsg)
{
  EXPECT_EQ(ros_twist2d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_twist2d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speed_along_x_body_axis,romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speed_along_y_body_axis,romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.angular_speed_around_z_body_axis,romea_twist2d.angularSpeed);
  isSame(ros_twist2d_msg.twist.covariance,romea_twist2d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRosMsgtoRomea)
{
  romea::Twist2D romea_tsist2d_bis = romea::toRomea(ros_twist2d_msg.twist);

  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.linearSpeeds.x(),romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.linearSpeeds.y(),romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.angularSpeed,romea_twist2d.angularSpeed);
  isSame(romea_tsist2d_bis.covariance,romea_twist2d.covariance);

}

//-----------------------------------------------------------------------------
class TestTwist3DConversion : public ::testing::Test
{
public :

  TestTwist3DConversion():
    stamp(1000),
    frame_id("foo"),
    romea_twist3d(),
    ros_twist3d_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_twist3d.linearSpeeds.x()=1;
    romea_twist3d.linearSpeeds.y()=2;
    romea_twist3d.linearSpeeds.z()=3;
    romea_twist3d.angularSpeeds.z()=4;
    romea_twist3d.angularSpeeds.x()=5;
    romea_twist3d.angularSpeeds.z()=6;
    fillEigenCovariance(romea_twist3d.covariance);
    romea::toRosMsg(stamp,frame_id,romea_twist3d,ros_twist3d_msg);
  }

  ros::Time stamp;
  std::string frame_id;
  romea::Twist3D romea_twist3d;
  geometry_msgs::TwistWithCovarianceStamped ros_twist3d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist3DConversion, fromRomeatoRosMsg)
{
  EXPECT_EQ(ros_twist3d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_twist3d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.x,romea_twist3d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.y,romea_twist3d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.z,romea_twist3d.linearSpeeds.z());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.x,romea_twist3d.angularSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.y,romea_twist3d.angularSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.z,romea_twist3d.angularSpeeds.z());
  isSame(ros_twist3d_msg.twist.covariance,romea_twist3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestTwist3DConversion, fromRosMsgtoRomea)
{
  romea::Twist3D romea_tsist3d_bis = romea::toRomea(ros_twist3d_msg.twist);

  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.x(),romea_twist3d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.y(),romea_twist3d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.z(),romea_twist3d.linearSpeeds.z());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.x(),romea_twist3d.angularSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.y(),romea_twist3d.angularSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.z(),romea_twist3d.angularSpeeds.z());
  isSame(romea_tsist3d_bis.covariance,romea_twist3d.covariance);
}

//-----------------------------------------------------------------------------
class TestPose2DConversion : public ::testing::Test
{
public :

  TestPose2DConversion():
    stamp(1000),
    frame_id("foo"),
    romea_pose2d(),
    ros_pose2d_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_pose2d.position.x()=1;
    romea_pose2d.position.y()=2;
    romea_pose2d.yaw=3;
    fillEigenCovariance(romea_pose2d.covariance);
    romea::toRosMsg(stamp,frame_id,romea_pose2d,ros_pose2d_msg);
  }

  ros::Time stamp;
  std::string frame_id;
  romea::Pose2D romea_pose2d;
  romea_localisation_msgs::Pose2DStamped ros_pose2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPose2DConversion, fromRomeaToRosMsg)
{
  EXPECT_EQ(ros_pose2d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_pose2d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.x,romea_pose2d.position.x());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.y,romea_pose2d.position.y());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.yaw,romea_pose2d.yaw);
  isSame(ros_pose2d_msg.pose.covariance,romea_pose2d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose2DConversion, fromRosMsgToRomea)
{
  romea::Pose2D romea_pose2d_bis = romea::toRomea(ros_pose2d_msg.pose);

  EXPECT_DOUBLE_EQ(romea_pose2d_bis.position.x(),romea_pose2d.position.x());
  EXPECT_DOUBLE_EQ(romea_pose2d_bis.position.y(),romea_pose2d.position.y());
  EXPECT_DOUBLE_EQ(romea_pose2d_bis.yaw,romea_pose2d.yaw);
  isSame(romea_pose2d_bis.covariance,romea_pose2d.covariance);
}

//-----------------------------------------------------------------------------
class TestPose3DConversion : public ::testing::Test
{
public :

  TestPose3DConversion():
    stamp(1000),
    frame_id("foo"),
    child_frame_id("bar"),
    romea_pose3d(),
    quaternion(),
    ros_pose3d_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_pose3d.position.x()=1;
    romea_pose3d.position.y()=2;
    romea_pose3d.position.z()=3;
    romea_pose3d.orientation.x()=0.1;
    romea_pose3d.orientation.y()=0.2;
    romea_pose3d.orientation.z()=0.3;
    fillEigenCovariance(romea_pose3d.covariance);
    romea::toRosMsg(stamp,frame_id,romea_pose3d,ros_pose3d_msg);
    quaternion=romea::eulerAnglesToRotation3D(romea_pose3d.orientation);
  }

  ros::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  romea::Pose3D romea_pose3d;
  Eigen::Quaterniond quaternion;
  geometry_msgs::PoseWithCovarianceStamped ros_pose3d_msg;

};

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, fromRomeaToRosMsg)
{
  EXPECT_EQ(ros_pose3d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_pose3d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.x,romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.y,romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.z,romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.x,quaternion.x());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.y,quaternion.y());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.z,quaternion.z());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.w,quaternion.w());
  isSame(ros_pose3d_msg.pose.covariance,romea_pose3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, fromRosMsgToRomea)
{
  romea::Pose3D romea_pose3d_bis = romea::toRomea(ros_pose3d_msg.pose);

  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.x(),romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.y(),romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.z(),romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.x(),romea_pose3d.orientation.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.y(),romea_pose3d.orientation.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.z(),romea_pose3d.orientation.z());
  isSame(romea_pose3d_bis.covariance,romea_pose3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, fromRomeaToRosTransformMsg)
{
  geometry_msgs::TransformStamped tf_msg;

  romea::toRosTransformMsg(stamp,romea_pose3d,frame_id,child_frame_id,tf_msg);

  EXPECT_EQ(tf_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(tf_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_STREQ(tf_msg.child_frame_id.c_str(),child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.x,romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.y,romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.z,romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.x,quaternion.x());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.y,quaternion.y());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.z,quaternion.z());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.w,quaternion.w());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
