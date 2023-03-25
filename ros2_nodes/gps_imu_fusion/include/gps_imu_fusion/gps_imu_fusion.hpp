#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

#include <InsFilterNonHolonomic.hpp>

#include <fstream>

using namespace std;
using namespace Eigen;

class GpsImuFusion : public rclcpp::Node
{
private:
    /* Bool to check if filter is initialized */
    bool is_filter_orientation_initialized = false;
    bool is_filter_location_initialized = false;
    bool is_ref_location_set = false;
    
    /* Bool to understand if vehicle is still and so if filter doesn't need to be updated */
    bool is_vehicle_still = true;

    /* Velocity parameters */
    double total_dist = 0.0;
    double act_dist = 0.0;

    /* Sensors Frequency (Hz) */
    uint8_t imu_fs, gps_fs;

    /* Noises Parameters */
    std::vector<double> gyroscope_noise;
    std::vector<double> gyroscope_bias_noise;
    std::vector<double> accelerometer_noise;
    std::vector<double> accelerometer_bias_noise;
    double r_vel = 0.0f;
    double r_pos = 0.005f;
    double gyroscope_bias_decay_factor = 1.0f;
    double accel_bias_decay_factor = 1.0f;
    double zero_velocity_constraint_noise = 1e-3;
    double gps_acc_unc = 0.005;

    /**
     * TODO: Create log files
     */
    uint16_t count_imu_topics = 0;
    uint16_t count_gps_topics = 0;
    uint16_t count_total_topic = 0;
    Vector3d prev_loc;
    ofstream predictFile;
    double vx = 0;
    double vy = 0;


    /* IMU and GPS ROS 2 topics */
    string imu_topic, gps_topic; 

    /* IMU and GPS EKF */
    InsFilterNonHolonomic* gndFusion;

    /* IMU data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    /* GPS data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;

    /* TF2 broadcaster */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* Odometry publisher */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    /**
     * Load ROS 2 parameters function
    */
    void loadParameters();

    /**
     * IMU data callback
    */
    void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);

    /**
     * GPS data callback
    */
    void gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data);

    /**
     * Update TF Pose
     * 
     * @param[in] p Vehicle position
     * @param[in] q Vehicle orientation
    */
    void updateTf(Vector3d p, tf2::Quaternion q);

public:
    /* Default constructor */
    GpsImuFusion();
};