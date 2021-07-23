//gtest
#include <gtest/gtest.h>

//ros
#include <ros/ros.h>

//romea
#include "romea_common_utils/conversions/transform_conversions.hpp"

TEST(TestRosTransform, lookupTransformOnce)
{
  ros::NodeHandle n;
  Eigen::Affine3d transform;

  EXPECT_NO_THROW({transform=romea::lookupTransformOnce("a","b",
                   ros::Time::now(),
                   ros::Duration(10));});

  EXPECT_NEAR(transform.linear()(0,0),1.,0.000001);
  EXPECT_NEAR(transform.linear()(1,1),1.,0.000001);
  EXPECT_NEAR(transform.linear()(2,2),1.,0.000001);

  EXPECT_NEAR(transform.translation().x(),1.,0.000001);
  EXPECT_NEAR(transform.translation().y(),2.,0.000001);
  EXPECT_NEAR(transform.translation().z(),3.,0.000001);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_param_test");
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
