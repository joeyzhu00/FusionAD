#ifndef FRAME_CALIBRATION_H
#define FRAME_CALIBRATION_H

/*
NOTE: This node listens to the tf broadcasts and performs the homogeneous transforms.
      Another major function of this node is to "calibrate" the initial position and orientation
      of the vehicle. The homogeneous transforms are applied and then the result is published.
      Also contains logic to filter out GPS points that are outside of a certain threshold which is
      calculated by taking the distance between each GPS point. 

NOTE: The covariance in the eth-zurich swift package provides a filtered gps signal and a non-filtered gps signal.
      A covariance of 1 [m^2] is typically done with their "single point estimation" and a covariance of 0.0049 and 
      25 [m^2] has been filtered. Because the 1 [m^2] covariance is typically noisy due to the lack of filtering and the 
      25 [m^2] covariance is filtered and smooth, the two covariances are swapped. This allows the EKF to take in the 
      previously 25 [m^2] measurements and apply a larger Kalman Gain (trusting this measurement more). 
      
Subscribers
--------------------------------------------------
Topic:  /gps/geodesy_odom
            Msg: nav_msgs::Odometry
Topic:  /imu
            Msg: sensor_msgs::Imu
Topic:  /localization/loam_odom_with_covar
            Msg: nav_msgs::Odometry
Topic:  /localization/wheel_odom
            Msg: nav_msgs::Odometry

TF Listeners: Geodesy TF Message
              Lidar TF Message

Publishers
-------------------------------------------------
Topic: /localization/calibration
            Msg: geometry_msgs::Pose
Topic: /localization/rotated_imu
            Msg: sensor_msgs::Imu
Topic: /localization/geodesy_tf
            Msg: nav_msgs::Odometry
Topic: /localization/lidar_tf
            Msg: nav_msgs::Odometry
*/


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <queue>
#include <algorithm>

namespace fusionad
{
namespace localization
{
namespace frame_calibration_node
{
class FrameCalibrationNode
{
    public:
        FrameCalibrationNode();
        ~FrameCalibrationNode();
        void initRosComm();

    private:

        // booleans to know when calibration is finished
        bool yaw_is_calibrated = false;
        bool geodesy_is_calibrated = false;

        // counters to keep track of # of samples
        // unsigned int yaw_samples_counter = 0;
        // unsigned int geodesy_samples_counter = 0;

        // variables to store samples
        float yaw_accumulation = 0; 
        // float geodesy_x_accumulation = 0;
        // float geodesy_y_accumulation = 0;
        float gps_heading_accumulation = 0;
        float gps_calibration_heading = 0;

        // float geodesy_calibrated_x_value = 0;
        // float geodesy_calibrated_y_value = 0;

        // initializing nodehandle
        ros::NodeHandle frameCalibrationNode_nh;
        
        // initialize publishers
        ros::Publisher calibrated_pose_pub;

        // publish transformed measurements
        ros::Publisher geodesy_tf_pub;
        ros::Publisher lidar_tf_pub;
        ros::Publisher imu_tf_pub;

        // initialize subscribers
        ros::Subscriber geodesy_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber odom_sub;

        // inputs to the calibration message
        float calibrated_yaw;
        sensor_msgs::Imu rot_msg;

        // messages after transform
        nav_msgs::Odometry geodesy_tf_msg;
        sensor_msgs::Imu imu_tf_msg;
        nav_msgs::Odometry previous_geodesy_tf_msg;

        float previous_vehicle_heading;
        geometry_msgs::Point previous_geodesy_point;

        bool first_message_sent = false;

        nav_msgs::Odometry lidar_tf_msg;
        
        tf::TransformListener geodesy_listener;
        tf::TransformListener lidar_listener;

        bool speed_threshold_met = false;
        bool calibration_in_progress = true;
        
        // message threshold for calibration
        const int MSG_THRESHOLD = 100;
        unsigned int heading_sample_counter = 0;
        
        // declaring callbacks
        void geodesyCallback(const nav_msgs::Odometry& geodesy_msg);
        void yawCallback(const sensor_msgs::Imu& imu_msg);
        void lidarCallback(const nav_msgs::Odometry& lidar_msg);
        void odomCallback(const nav_msgs::Odometry& odom_msg);

        // support functions
        void covarianceRescaling(const nav_msgs::Odometry& geodesy_msg);
        float headingCalibration(float gps_heading);
        void backwardDiffHeading(const nav_msgs::Odometry& geodesy_msg);
        // position calibration functions
        void gpsPoseCalibration();
};
}// frame_calibration_node
}// localization
}// fusionad


#endif