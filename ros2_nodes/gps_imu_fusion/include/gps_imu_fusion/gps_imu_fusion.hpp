#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <InsFilterNonHolonomic.hpp>

using namespace std;
using namespace Eigen;

class GpsImuFusion : public rclcpp::Node
{
private:
    /* Bool to check if filter is initialized */
    bool is_filter_initialized = false;

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

    /* IMU and GPS ROS 2 topics */
    string imu_topic, gps_topic; 

    /* IMU and GPS EKF */
    InsFilterNonHolonomic* gndFusion;

    /* IMU data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    /* GPS data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;

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


public:
    /* Default constructor */
    GpsImuFusion();
};