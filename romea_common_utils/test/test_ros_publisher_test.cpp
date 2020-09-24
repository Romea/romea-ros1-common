//gtest
#include <gtest/gtest.h>

//romea
#include "test_ros_publisher_utils.hpp"
#include "romea_common_utils/conversions/TransformConversions.hpp"
#include "romea_common_utils/conversions/DiagnosticConversions.hpp"
#include "romea_common_utils/publishers/MessagePublisher.hpp"
#include "romea_common_utils/publishers/StampedMessagePublisher.hpp"
#include "romea_common_utils/publishers/OdomPublisher.hpp"
#include "romea_common_utils/publishers/TransformPublisher.hpp"
#include "romea_common_utils/publishers/DiagnosticPublisher.hpp"


TEST(TestRosPublisher, testMessagePublisher)
{
  ros::NodeHandle nh;
  AnyHelper<std_msgs::String> h;
  romea::MessagePublisher<std::string,std_msgs::String> pub(nh,"foo",0);
  ros::Subscriber sub = nh.subscribe("foo", 0, &AnyHelper<std_msgs::String>::cb, &h);

  pub.publish("bar");
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_STREQ(h.data.data.c_str(),"bar");
}


TEST(TestRosPublisher, testStampedMessagePublisher)
{
  ros::NodeHandle nh;
  AnyHelper<geometry_msgs::PointStamped> h;
  romea::StampedMessagePublisher<Eigen::Vector3d,geometry_msgs::PointStamped> pub(nh,"foo","bar",0);
  ros::Subscriber sub = nh.subscribe("foo", 0, &AnyHelper<geometry_msgs::PointStamped>::cb, &h);

  ros::Time t= ros::Time::now();
  pub.publish(t,Eigen::Vector3d(1,2,3));
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(h.data.header.stamp.toNSec(),t.toNSec());
  EXPECT_STREQ(h.data.header.frame_id.c_str(),"bar");
  EXPECT_DOUBLE_EQ(h.data.point.x,1);
  EXPECT_DOUBLE_EQ(h.data.point.y,2);
  EXPECT_DOUBLE_EQ(h.data.point.z,3);
}


TEST(TestRosPublisher, testOdomPublisher)
{
  ros::NodeHandle nh;
  AnyHelper<nav_msgs::Odometry> h;
  romea::OdomPublisher<nav_msgs::Odometry> pub(nh,"odom","foo","bar",0);
  ros::Subscriber sub = nh.subscribe("odom", 0, &AnyHelper<nav_msgs::Odometry>::cb, &h);

  ros::Time t= ros::Time::now();
  pub.publish(t,nav_msgs::Odometry());
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(h.data.header.stamp.toNSec(),t.toNSec());
  EXPECT_STREQ(h.data.header.frame_id.c_str(),"foo");
  EXPECT_STREQ(h.data.child_frame_id.c_str(),"bar");

}

TEST(TestRosPublisher, testTransformPublisher)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  romea::TransformPublisher<Eigen::Affine3d> pub("foo","bar");

  auto t = ros::Time::now();
  pub.publish(t-ros::Duration(0.1),Eigen::Affine3d::Identity());
  ros::spinOnce();
  pub.publish(t+ros::Duration(0.1),Eigen::Affine3d::Identity());
  ros::spinOnce();

  EXPECT_TRUE(tf_buffer.canTransform("bar","foo",t,ros::Duration(1)));
}

TEST(TestRosPublisher, testDiagnosticPublisher)
{
  ros::NodeHandle nh;
  AnyHelper<diagnostic_msgs::DiagnosticArray> h;
  romea::DiagnosticPublisher<romea::DiagnosticReport> pub(nh,"foo");
  ros::Subscriber sub = nh.subscribe("/diagnostics", 0, &AnyHelper<diagnostic_msgs::DiagnosticArray>::cb, &h);

  ros::Time t= ros::Time::now();
  romea::DiagnosticReport report;
  report.diagnostics.push_back(romea::Diagnostic(romea::DiagnosticStatus::ERROR,"bar"));
  report.info["bar"]="error";
  pub.publish(t,report);
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(h.data.header.stamp.toNSec(),t.toNSec());
  EXPECT_STREQ(h.data.status[0].name.c_str(),"foo");
  EXPECT_EQ(h.data.status[0].level,diagnostic_msgs::DiagnosticStatus::ERROR);
  EXPECT_STREQ(h.data.status[0].values[0].key.c_str(),"bar");
  EXPECT_STREQ(h.data.status[0].values[0].value.c_str(),"error");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_param_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
