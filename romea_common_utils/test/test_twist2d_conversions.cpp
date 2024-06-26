// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    romea::to_ros_msg(stamp,frame_id,romea_twist2d,ros_twist2d_msg);
  }

  ros::Time stamp;
  std::string frame_id;
  romea::core::Twist2D romea_twist2d;
  romea_common_msgs::Twist2DStamped ros_twist2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRomeaToRosMsg)
{
  EXPECT_EQ(ros_twist2d_msg.header.stamp.toNSec(),stamp.toNSec());
  EXPECT_STREQ(ros_twist2d_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.x,romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.y,romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.angular_speed,romea_twist2d.angularSpeed);
  isSame(ros_twist2d_msg.twist.covariance,romea_twist2d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRosMsgto_romea)
{
  romea::core::Twist2D romea_tsist2d_bis = romea::to_romea(ros_twist2d_msg.twist);

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
