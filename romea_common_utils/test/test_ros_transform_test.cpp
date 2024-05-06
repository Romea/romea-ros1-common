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
