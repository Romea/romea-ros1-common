// gtest
#include <gtest/gtest.h>

//romea
#include "romea_common_utils/conversions/TransformConversions.hpp"
#include "romea_common_utils/conversions/GeometryConversions.hpp"
#include "romea_common_utils/conversions/DiagnosticConversions.hpp"

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testvector3Conversion)
{
  {
    Eigen::Vector3d vector3(1.2,-3.5,10.);
    geometry_msgs::Vector3 vector3_msg;
    romea::toRosMsg(vector3,vector3_msg);

    EXPECT_DOUBLE_EQ(vector3.x(),vector3_msg.x);
    EXPECT_DOUBLE_EQ(vector3.y(),vector3_msg.y);
    EXPECT_DOUBLE_EQ(vector3.z(),vector3_msg.z);
  }

  {
    geometry_msgs::Vector3 vector3_msg;
    vector3_msg.x=13.3;
    vector3_msg.y=6.8;
    vector3_msg.z=-6;

    Eigen::Vector3d vector3;
    romea::toRomea(vector3_msg,vector3);

    EXPECT_DOUBLE_EQ(vector3.x(),vector3_msg.x);
    EXPECT_DOUBLE_EQ(vector3.y(),vector3_msg.y);
    EXPECT_DOUBLE_EQ(vector3.z(),vector3_msg.z);
  }

}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testQuaternionConversion)
{
  double roll=0.3;
  double pitch=-0.9;
  double yaw= -2.5;

  Eigen::Quaterniond quaternion=
      Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());

  geometry_msgs::Quaternion quaternion_msg;
  romea::toRosMsg(quaternion,quaternion_msg);

  EXPECT_DOUBLE_EQ(quaternion.x(),quaternion_msg.x);
  EXPECT_DOUBLE_EQ(quaternion.y(),quaternion_msg.y);
  EXPECT_DOUBLE_EQ(quaternion.z(),quaternion_msg.z);
  EXPECT_DOUBLE_EQ(quaternion.w(),quaternion_msg.w);

  romea::toRomea(quaternion_msg,quaternion);
  EXPECT_DOUBLE_EQ(quaternion_msg.x,quaternion.x());
  EXPECT_DOUBLE_EQ(quaternion_msg.y,quaternion.y());
  EXPECT_DOUBLE_EQ(quaternion_msg.z,quaternion.z());
}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testTransformConversion)
{
  double x= -4.4;
  double y= 2.4;
  double z= 20.3;
  double roll=0.4;
  double pitch=1.2;
  double yaw= 3.2;

  Eigen::Quaterniond quaternion=
      Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());

  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  Eigen::Affine3d transform;
  transform.linear()= rotation;
  transform.translation()= Eigen::Vector3d(x,y,z);

  geometry_msgs::Transform transform_msg;
  romea::toRosTransformMsg(transform,transform_msg);
  EXPECT_DOUBLE_EQ(transform_msg.translation.x,x);
  EXPECT_DOUBLE_EQ(transform_msg.translation.y,y);
  EXPECT_DOUBLE_EQ(transform_msg.translation.z,z);
  EXPECT_DOUBLE_EQ(transform_msg.rotation.x,quaternion.x());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.y,quaternion.y());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.z,quaternion.z());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.w,quaternion.w());

  romea::toRomea(transform_msg,transform);
  EXPECT_DOUBLE_EQ(transform.translation().x(),x);
  EXPECT_DOUBLE_EQ(transform.translation().y(),y);
  EXPECT_DOUBLE_EQ(transform.translation().z(),z);
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      EXPECT_NEAR(transform.linear()(i,j),rotation(i,j),0.00001);
    }
  }
}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testDiagnosticConversion)
{

  romea::DiagnosticReport report;
  report.diagnostics.push_back(romea::Diagnostic(romea::DiagnosticStatus::OK,"foo"));
  report.diagnostics.push_back(romea::Diagnostic(romea::DiagnosticStatus::ERROR,"bar"));
  report.info["foo"]="valid";
  report.info["bar"]="empty";

  diagnostic_msgs::DiagnosticStatus status;
  romea::toRosDiagnosticMsg("baz","qux",report,status);

  EXPECT_STREQ(status.name.c_str(),"baz");
  EXPECT_STREQ(status.hardware_id.c_str(),"qux");
  EXPECT_EQ(status.level,diagnostic_msgs::DiagnosticStatus::ERROR);
  EXPECT_STREQ(status.message.c_str(),"foo bar ");
  EXPECT_STREQ(status.values[0].key.c_str(),"bar");
  EXPECT_STREQ(status.values[0].value.c_str(),"empty");
  EXPECT_STREQ(status.values[1].key.c_str(),"foo");
  EXPECT_STREQ(status.values[1].value.c_str(),"valid");

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
