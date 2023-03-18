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

    this->predictFile.open("/tmp/predict_data.txt");

    /* Initialize the transform broadcaster */
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
        this->gndFusion->setInitState(
            Vector4d(qw, qx, qy, qz),   /* Initial Quaternion */
            Vector3d(0.000265955156806132, 0.000118613066929290, -0.000865860620781665), 
            Vector3d::Zero(), 
            Vector3d::Zero(), 
            Vector3d(9.42558485122576e-05, -2.35931658257626e-05, 0.000160709506499922)
        );
        this->is_filter_initialized = true;
        RCLCPP_INFO(this->get_logger(), "Inital state covariance:");
        this->gndFusion->printCurrentStateCovariance();
    }

    RCLCPP_INFO(this->get_logger(), "Current filter state:");
    this->gndFusion->printCurrentState();
    
    /* Convert IMU accel_data to eigen vector */
    Vector3f accel_data(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);

    /* Estimate vx and vy */
    // vx += (imu_data->linear_acceleration.x < 0) ? ((-imu_data->linear_acceleration.x)*0.01) : (imu_data->linear_acceleration.x*0.01);
    // vy += (imu_data->linear_acceleration.y < 0) ? ((-imu_data->linear_acceleration.y)*0.01) : (imu_data->linear_acceleration.y*0.01);

    // if ((this->count_imu_topics % 10) == 0)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Estimated Vx (ms): %f Estimated Vy (ms): %f", vx, vy);
    //     vx = 0;
    //     vy = 0;
    // }

    Vector3f gyro_data(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);
    
    RCLCPP_INFO(this->get_logger(), "PREDICTING NEW SYSTEM STATE");
    /* Predict */
    this->gndFusion->predict(accel_data, gyro_data);
    end = this->now();
    rclcpp::Duration exe_time = end - start;

    RCLCPP_INFO(this->get_logger(), "Predict exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);

    Vector4d act_orientation;
    Vector3d act_postion;
    Vector3d act_velocities;

    this->gndFusion->pose(act_postion, act_orientation, act_velocities);

    this->count_imu_topics++;

    if (this->count_imu_topics <= 100)
    {
        // Write to the file
        this->predictFile << "ITER ["<<(count_imu_topics+'0')<<"]: " << "\n"
            << "qw: " << act_orientation[0] << "\n"
            << "qx: " << act_orientation[1] << "\n"
            << "qy: " << act_orientation[2] << "\n"
            << "qz: " << act_orientation[3] << "\n";
    } else {
        // Close the file
        predictFile.close();
    }

    /* Update vehicle TF */
    geometry_msgs::msg::Quaternion q;
    q.w = act_orientation[0];
    q.x = act_orientation[1];
    q.y = act_orientation[2];
    q.z = act_orientation[3];
    this->updateTf(act_postion, q);
}

void GpsImuFusion::gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data)
{
    // RCLCPP_INFO(this->get_logger(), "Recv GPS data");
}

/* ***** END CALLBACKS ***** */
void GpsImuFusion::updateTf(Vector3d p, geometry_msgs::msg::Quaternion q)
{
    geometry_msgs::msg::TransformStamped t;

    /* Update header */
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "vehicle";

    /* Update TF location */
    t.transform.translation.x = p[0];
    t.transform.translation.y = p[1];
    t.transform.translation.z = 0.0f;

    /* Update TF Orientation */
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    t.transform.rotation.w = q.w;

    /* Publish new position */
    tf_broadcaster_->sendTransform(t);
}