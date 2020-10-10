//gtest
#include <gtest/gtest.h>

//ros
#include <ros/ros.h>

//romea
#include "romea_common_utils/params/RosParam.hpp"
#include "romea_common_utils/params/RosEigenParam.hpp"

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
  romea::GeodeticCoordinates geodetic_coordinates;

  EXPECT_NO_THROW({geodetic_coordinates=romea::loadGeodeticCoordinates(private_nh,"geodetic");});

  EXPECT_NEAR(geodetic_coordinates.getLatitude(),45.85207/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.getLongitude(),3.16482/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.getAltitude(),300.0,0.000001);
}

TEST(TestRosParams, loadMapFloat)
{
  ros::NodeHandle private_nh("~");
  std::map<std::string,double> map ;

  EXPECT_NO_THROW({map=romea::loadMap<double>(private_nh,"map_float");});
  EXPECT_DOUBLE_EQ(map["foo"],0.34);
  EXPECT_DOUBLE_EQ(map["bar"],-2.7);
  EXPECT_DOUBLE_EQ(map["baz"],5.5);
}

TEST(TestRosParams, loadUnavailableMap)
{
  ros::NodeHandle private_nh("~");
  std::map<std::string,double> map ;
  EXPECT_THROW({map=romea::loadMap<double>(private_nh,"unavailable_float");},std::runtime_error);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_param_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
