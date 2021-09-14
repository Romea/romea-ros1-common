// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_common_utils/conversions/pose2d_conversions.hpp"

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
  romea_common_msgs::Pose2DStamped ros_pose2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPose2DConversion, fromRomeaToRosMsg)
{
  EXPECT_EQ(ros_pose2d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_pose2d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.position.x,romea_pose2d.position.x());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.position.y,romea_pose2d.position.y());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.yaw,romea_pose2d.yaw);
  isSame(ros_pose2d_msg.pose.covariance,romea_pose2d.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}