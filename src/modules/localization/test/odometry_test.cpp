#include <localization/wheel_odom/odometry.h>
#include <gtest/gtest.h>
#include <stdio.h>
/*
GTest for the odometry.cpp script

Includes the followings tests:
------------------------------
TEST CASE 1: odometry_state_estimation()
REASON     : Check whether 0 movement results in x-position of 0

TEST CASE 2: odometry_state_estimation()
REASON     : Check whether 0 movement results in y-position of 0

TEST CASE 3: odometry_state_estimation()
REASON     : Check whether 0 yaw value results in y-position of 0

TEST CASE 4: odometry_state_estimation()
REASON     : Check whether 0 yaw value results in correct x-position

TEST CASE 5: odometry_state_estimation()
Reason     : Check whether the positive encoder messages result in correct x-position

TEST CASE 6: odometry_state_estimation()
Reason     : Check whether the positive encoder messages result in correct y-position

TEST CASE 7: wheel_odom_median_filter()
Reason     : Check whether median filter has correct output (input of 0)

TEST CASE 8: wheel_odom_median_filter()
Reason     : Check whether median filter has correct output
*/

//////////////////////////////////////////////////////////////
// TEST CASE 1: Testing the odometry_state_estimation logic //
// Checking for 0 movement input to the odometry logic      //
//////////////////////////////////////////////////////////////
    
TEST(odometry_state_test_1, ShouldPass)
{
    fusionad::localization::wheel_odometry_node::WheelOdometryNode odom_state_1;

    float left_angular_vel = 0;
    float right_angular_vel = 0;
    float yaw_estimate = 0;
    float previous_x_position = 0;
    float x_position = 0;

    odom_state_1.odometry_state_estimation();
    float TEST_1_SOLUTION = 0;

    ASSERT_NEAR(TEST_1_SOLUTION, x_position, std::abs(TEST_1_SOLUTION)*0.01);
}

//////////////////////////////////////////////////////////////
// TEST CASE 7: Testing the wheel_odom_median_filter logic ///
// Checking for 0 velocity input to the median filter      ///
//////////////////////////////////////////////////////////////

TEST(median_filter_test_1, ShouldPass)
{
    fusionad::localization::wheel_odometry_node::WheelOdometryNode median_filter_1;

    float vel_magnitude = 0;

    for(int j = 0; j < 10; j++)
    {
        median_filter_1.wheel_odom_median_filter();
    }

    float TEST_7_SOLUTION = 0;
    
    ASSERT_NEAR(TEST_7_SOLUTION, vel_magnitude, std::abs(TEST_7_SOLUTION)*0.01);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "OdometryTest");
    ros::NodeHandle odometry_test_nh;
    return RUN_ALL_TESTS();
}