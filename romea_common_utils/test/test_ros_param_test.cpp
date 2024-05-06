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
#include "romea_common_utils/params/ros_param.hpp"
#include "romea_common_utils/params/ros_eigen_param.hpp"


TEST(TestRosParams, loadParam)
{
  ros::NodeHandle private_nh("~");
  EXPECT_STREQ(romea::load_param<std::string>(private_nh,"load_param").c_str(),"foo");
}

TEST(TestRosParams, loadParamOrWhenValueIsAvailable)
{
  ros::NodeHandle private_nh("~");
  EXPECT_STREQ(romea::load_param_or<std::string>(private_nh,"load_param_or","bar").c_str(),"foo");
}

TEST(TestRosParams, loadParamOrWhenValueIsUnavailable)
{
  ros::NodeHandle private_nh("~");
  EXPECT_STREQ(romea::load_param_or<std::string>(private_nh,"unavailable_param","bar").c_str(),"bar");
}

TEST(TestRosParams, loadVector3d)
{
  ros::NodeHandle private_nh("~");
  Eigen::Vector3d vector3d;

  EXPECT_NO_THROW({vector3d=romea::loadEigenVector<Eigen::Vector3d>(private_nh,"vector3d");});

  EXPECT_NEAR(vector3d.x(),2.3,0.000001);
  EXPECT_NEAR(vector3d.y(),5.4,0.000001);
  EXPECT_NEAR(vector3d.z(),-8.9,0.000001);
}

TEST(TestRosParams, loadVector3i)
{
  ros::NodeHandle private_nh("~");
  Eigen::Vector3d vector3d;

  EXPECT_NO_THROW({vector3d= romea::loadEigenVector<Eigen::Vector3d>(private_nh,"vector3d");});

  EXPECT_NEAR(vector3d.x(),2.3,0.000001);
  EXPECT_NEAR(vector3d.y(),5.4,0.000001);
  EXPECT_NEAR(vector3d.z(),-8.9,0.000001);
}

TEST(TestRosParams, loadGeodeticCoordinate)
{
  ros::NodeHandle private_nh("~");
  romea::core::GeodeticCoordinates geodetic_coordinates;

  EXPECT_NO_THROW({geodetic_coordinates=romea::load_geodetic_coordinates(private_nh,"geodetic");});

  EXPECT_NEAR(geodetic_coordinates.latitude,45.85207/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.longitude,3.16482/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.altitude,300.0,0.000001);
}

TEST(TestRosParams, loadMapFloat)
{
  ros::NodeHandle private_nh("~");
  std::map<std::string,double> map ;

  EXPECT_NO_THROW({map=romea::load_map<double>(private_nh,"map_float");});
  EXPECT_DOUBLE_EQ(map["foo"],0.34);
  EXPECT_DOUBLE_EQ(map["bar"],-2.7);
  EXPECT_DOUBLE_EQ(map["baz"],5.5);
}

TEST(TestRosParams, loadUnavailableMap)
{
  ros::NodeHandle private_nh("~");
  std::map<std::string,double> map ;
  EXPECT_THROW({map=romea::load_map<double>(private_nh,"unavailable_float");},std::runtime_error);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_param_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
