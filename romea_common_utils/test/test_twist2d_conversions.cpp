// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

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
  romea_common_msgs::Twist2DStamped ros_twist2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRomeatoRosMsg)
{
  EXPECT_EQ(ros_twist2d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_twist2d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.x,romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.y,romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.angular_speed,romea_twist2d.angularSpeed);
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
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}