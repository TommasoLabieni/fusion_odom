#include <gps_imu_fusion/gps_imu_fusion.hpp>

GpsImuFusion::GpsImuFusion() : rclcpp::Node("gps_imu_fusion_node")
{
    /* Load ROS 2 parameters */
    this->loadParameters();

    /* Create subscribers */
	this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
	    this->imu_topic, 10, std::bind(&GpsImuFusion::imuDataCallback, this, std::placeholders::_1)
    );

	this->gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
	    this->gps_topic, 10, std::bind(&GpsImuFusion::gpsDataCallback, this, std::placeholders::_1)
    );

    /* Create EKF Filter */
    this->gndFusion = new InsFilterNonHolonomic();

    /* Load initialia parameters to the filter */
    this->gndFusion->loadParameters(Vector3f(this->gyroscope_noise[0], this->gyroscope_noise[1],this->gyroscope_noise[2]),
        Vector3f(this->gyroscope_bias_noise[0], this->gyroscope_bias_noise[1], this->gyroscope_bias_noise[2]),
        Vector3f(this->accelerometer_noise[0], this->accelerometer_noise[1], this->accelerometer_noise[2]),
        Vector3f(this->accelerometer_bias_noise[0], this->accelerometer_bias_noise[1], this->accelerometer_bias_noise[2]),
        this->r_vel, 
        this->r_pos,
        this->gyroscope_bias_decay_factor,
        this->accel_bias_decay_factor
    );

    RCLCPP_INFO(this->get_logger(), "***** INITIAL FILTER CONSTRAINTS *****");
    this->gndFusion->printFilterConstraints();
    RCLCPP_INFO(this->get_logger(), "**************************************");

}

void GpsImuFusion::loadParameters()
{
	declare_parameter("generic.imu_topic", "/imu/data");
	declare_parameter("generic.gps_topic", "/gps/position");

    /* Declare Sensor Frequency parameters */
	declare_parameter("frequency.imu_fs", 100);
	declare_parameter("frequency.gps_fs", 10);

    /* Declare Sensor Noise parameters */
	declare_parameter("noises.gyro_noise", {});
	declare_parameter("noises.gyro_bias_noise", {});
	declare_parameter("noises.accel_noise", {});
	declare_parameter("noises.accel_bias_noise", {});
	declare_parameter("noises.r_vel", 0.0);
	declare_parameter("noises.r_pos", 0.005);
	declare_parameter("noises.gyroscope_bias_decay_factor", 1.0);
	declare_parameter("noises.accel_bias_decay_factor", 1.0);

    get_parameter("generic.imu_topic", this->imu_topic);
    get_parameter("generic.gps_topic", this->gps_topic);

    /* Get Sensor Frequency parameters */
	get_parameter("frequency.imu_fs", this->imu_fs);
	get_parameter("frequency.gps_fs", this->gps_fs);

    /* Get Sensor Noise parameters */
	get_parameter("noises.gyro_noise", this->gyroscope_noise);
	get_parameter("noises.gyro_bias_noise", this->gyroscope_bias_noise);
	get_parameter("noises.accel_noise", this->accelerometer_noise);
	get_parameter("noises.accel_bias_noise", this->accelerometer_bias_noise);
	get_parameter("noises.r_vel", this->r_vel);
	get_parameter("noises.r_pos", this->r_pos);
	get_parameter("noises.gyroscope_bias_decay_factor", this->gyroscope_bias_decay_factor);
	get_parameter("noises.accel_bias_decay_factor", this->accel_bias_decay_factor);

}

/* ***** CALLBACKS ***** */

void GpsImuFusion::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
{
    rclcpp::Time start, end;
    start = this->now();
    RCLCPP_INFO(this->get_logger(), "Recv IMU data");

    double qx = imu_data->orientation.x;
    double qy = imu_data->orientation.y;
    double qz = imu_data->orientation.z;
    double qw = imu_data->orientation.w;

    if (!this->is_filter_initialized)
    {
        this->gndFusion->setInitState(Vector4d(qx, qy, qz, qw), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());
        this->is_filter_initialized = true;
    }

    RCLCPP_INFO(this->get_logger(), "Current filter state:");
    // this->gndFusion->printCurrentState();
    
    /* Convert IMU accel_data to eigen vector */
    Vector3f accel_data(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);
    Vector3f gyro_data(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);
    
    RCLCPP_INFO(this->get_logger(), "PREDICTING NEW SYSTEM STATE");
    /* Predict */
    this->gndFusion->predict(accel_data, gyro_data);
    // this->gndFusion->printCurrentState();
    end = this->now();
    rclcpp::Duration exe_time = end - start;
    RCLCPP_INFO(this->get_logger(), "Predict exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);
}

void GpsImuFusion::gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data)
{
    RCLCPP_INFO(this->get_logger(), "Recv GPS data");
}

/* ***** END CALLBACKS ***** */