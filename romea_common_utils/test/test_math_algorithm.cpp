// gtest
#include <gtest/gtest.h>

//romea
#include "math/Algorithm.hpp"


//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testSign)
{
  EXPECT_DOUBLE_EQ(romea::sign(-2.3),-1);
  EXPECT_DOUBLE_EQ(romea::sign(14.7), 1);
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testClamp)
{
  EXPECT_DOUBLE_EQ(romea::clamp(0.5,-1.,1.),0.5);
  EXPECT_DOUBLE_EQ(romea::clamp(1.5,-1.,1.),1.);
  EXPECT_DOUBLE_EQ(romea::clamp(-1.5,-1.,1.),-1.);
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testSymmetricClamp)
{
  EXPECT_DOUBLE_EQ(romea::symmetricClamp(0.5,1.),0.5);
  EXPECT_DOUBLE_EQ(romea::symmetricClamp(1.5,1.),1.);
  EXPECT_DOUBLE_EQ(romea::symmetricClamp(-1.5,1.),-1.);
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, safe_divide)
{
  EXPECT_DOUBLE_EQ(romea::safeDivide(1.,2.).get(),0.5);
  EXPECT_FALSE(romea::safeDivide(1.,0.).is_initialized());
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testSignedMin)
{
  EXPECT_DOUBLE_EQ(-10,romea::signedMin(-10.,10.));
  EXPECT_DOUBLE_EQ( -1,romea::signedMin(-1.,-10.));
  EXPECT_DOUBLE_EQ(  1,romea::signedMin( 1., 10.));
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testSignedFloor)
{
  EXPECT_DOUBLE_EQ(10,romea::signedFloor(10.4));
  EXPECT_DOUBLE_EQ(10,romea::signedFloor(10.6));
  EXPECT_DOUBLE_EQ(-10,romea::signedFloor(-10.4));
  EXPECT_DOUBLE_EQ(-10,romea::signedFloor(-10.6));
}

//-----------------------------------------------------------------------------
TEST(TestAlgorithm, testIsApproximatelyEqual)
{
   EXPECT_EQ(true,romea::isApproximatelyEqual(1.0, 1.0, 1e-10));
   EXPECT_EQ(false,romea::isApproximatelyEqual(-1.0, 1.0));
   EXPECT_EQ(true,romea::isApproximatelyEqual(-1.0, -1.0));
   EXPECT_EQ(true,romea::isApproximatelyEqual(1.0, 2.0/2.0));
   EXPECT_EQ(true,romea::isApproximatelyEqual(1000.0, 999.5, 1e-3));
   EXPECT_EQ(false,romea::isApproximatelyEqual(1000.0, 999.0, 1e-3));
}

//TEST(TestAlgorithm,testDefinitelyGreaterThan)
//{
//  EXPECT_EQ(true,romea::isDefinitelyGreaterThan(2.0, 1.0, 1e-10));
//  EXPECT_EQ(false,romea::isDefinitelyGreaterThan(1.0, 1.0));
//  EXPECT_EQ(false,romea::isDefinitelyGreaterThan(-2.0, 1.0));
//  EXPECT_EQ(true,romea::isDefinitelyGreaterThan(-0.5, -1.0));
//  EXPECT_EQ(true,romea::isDefinitelyGreaterThan(1000.0, 999.0, 1e-3));
//  EXPECT_EQ(true,romea::isDefinitelyGreaterThan(1000.0, 998.0, 1e-3));
//}

//TEST(TestAlgorithm, testIsDefinitelyLessThan)
//{
//  EXPECT_EQ(true,romea::isDefinitelyLessThan(1.0, 2.0, 1e-10));
//  EXPECT_EQ(false,romea::isDefinitelyLessThan(1.0, 1.0));
//  EXPECT_EQ(false,romea::isDefinitelyLessThan(1.0, -2.0));
//  EXPECT_EQ(true,romea::isDefinitelyLessThan(-1.0, -0.5));
//  EXPECT_EQ(true,romea::isDefinitelyLessThan(999.0,  1000.0, 1e-3));
//  EXPECT_EQ(true,romea::isDefinitelyLessThan(1000.0, 1000.0, 1e-3));
//}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
