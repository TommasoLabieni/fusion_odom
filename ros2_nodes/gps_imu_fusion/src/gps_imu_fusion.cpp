#include <gps_imu_fusion/gps_imu_fusion.hpp>

GpsImuFusion::GpsImuFusion() : rclcpp::Node("gps_imu_fusion_node")
{
    /* Load ROS 2 parameters */
    this->loadParameters();

    /* Create subscribers */
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic, 10, std::bind(&GpsImuFusion::imuDataCallback, this, std::placeholders::_1));

    this->gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->gps_topic, 10, std::bind(&GpsImuFusion::gpsDataCallback, this, std::placeholders::_1));

    /* Create EKF Filter */
    this->gndFusion = new InsFilterNonHolonomic();

    /* Load initialia parameters to the filter */
    this->gndFusion->loadParameters(Vector3d(this->gyroscope_noise[0], this->gyroscope_noise[1], this->gyroscope_noise[2]),
                                    Vector3d(this->gyroscope_bias_noise[0], this->gyroscope_bias_noise[1], this->gyroscope_bias_noise[2]),
                                    Vector3d(this->accelerometer_noise[0], this->accelerometer_noise[1], this->accelerometer_noise[2]),
                                    Vector3d(this->accelerometer_bias_noise[0], this->accelerometer_bias_noise[1], this->accelerometer_bias_noise[2]),
                                    this->r_vel,
                                    this->r_pos,
                                    this->gyroscope_bias_decay_factor,
                                    this->accel_bias_decay_factor,
                                    this->zero_velocity_constraint_noise);

    RCLCPP_INFO(this->get_logger(), "Initial constraints: \n");
    this->gndFusion->printFilterConstraints();

    this->predictFile.open("/tmp/predict_data.txt");

    /* Initialize the transform broadcaster */
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /* Create Odometry publisher */
    this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
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
    declare_parameter("noises.zero_velocity_constraint_noise", 1e-3);
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
    get_parameter("noises.zero_velocity_constraint_noise", this->zero_velocity_constraint_noise);
    get_parameter("noises.r_vel", this->r_vel);
    get_parameter("noises.r_pos", this->r_pos);
    get_parameter("noises.gyroscope_bias_decay_factor", this->gyroscope_bias_decay_factor);
    get_parameter("noises.accel_bias_decay_factor", this->accel_bias_decay_factor);
}

/* ***** CALLBACKS ***** */

void GpsImuFusion::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
{
    if (this->count_imu_topics == 10)
        return;
    rclcpp::Time start, end;
    start = this->now();

    double qx = imu_data->orientation.x;
    double qy = imu_data->orientation.y;
    double qz = imu_data->orientation.z;
    double qw = imu_data->orientation.w;

    if (!this->is_filter_initialized)
    {
        this->gndFusion->setInitState(
            Vector4d(qw, qx, qy, qz), /* Initial Quaternion */
            Vector3d::Zero(),//(0.000265955156806132, 0.000118613066929290, -0.000865860620781665),
            Vector3d::Zero(),
            Vector3d::Zero(),
            Vector3d::Zero());//(9.42558485122576e-05, -2.35931658257626e-05, 0.000160709506499922));
        this->is_filter_initialized = true;
        RCLCPP_INFO(this->get_logger(), "Inital state covariance:");
        // this->gndFusion->printCurrentStateCovariance();
    }

    /* Convert IMU accel_data to eigen vector */
    Vector3d accel_data(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);

    /* Estimate vx and vy */
    // vx += (imu_data->linear_acceleration.x < 0) ? ((-imu_data->linear_acceleration.x)*0.01) : (imu_data->linear_acceleration.x*0.01);
    // vy += (imu_data->linear_acceleration.y < 0) ? ((-imu_data->linear_acceleration.y)*0.01) : (imu_data->linear_acceleration.y*0.01);

    // if ((this->count_imu_topics % 10) == 0)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Estimated Vx (ms): %f Estimated Vy (ms): %f", vx, vy);
    //     vx = 0;
    //     vy = 0;
    // }

    Vector3d gyro_data(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);

    RCLCPP_INFO(this->get_logger(), "PREDICTING NEW SYSTEM STATE");
    std::cerr << "accel_data: " << accel_data << "\n";
    std::cerr << "gyro_data: " << gyro_data << "\n";
    /* Predict */
    this->gndFusion->predict(accel_data, gyro_data);
    end = this->now();
    rclcpp::Duration exe_time = end - start;

    RCLCPP_INFO(this->get_logger(), "Predict exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);

    RCLCPP_INFO(this->get_logger(), "Current filter state AFTER PREDICTION:");
    // this->gndFusion->printCurrentState();

    Vector4d act_orientation;
    Vector3d act_postion;
    Vector3d act_velocities;

    this->gndFusion->pose(act_postion, act_orientation, act_velocities);

    this->count_imu_topics++;

    if (this->count_imu_topics <= 6000)
    {
        // Write to the file
        this->predictFile << "ITER [" << (count_imu_topics + "") << "]: "
                          << "\n"
                          << "qw: " << act_orientation[0] << "\n"
                          << "qx: " << act_orientation[1] << "\n"
                          << "qy: " << act_orientation[2] << "\n"
                          << "qz: " << act_orientation[3] << "\n"
                          << "pn: " << act_postion[0] << "\n"
                          << "pe: " << act_postion[1] << "\n"
                          << "pd: " << act_postion[2] << "\n";
    }
    else
    {
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

Vector3d measure_distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2, double time)
{
    Vector3d velocities;
    double dLat = lat2 * M_PI / 180.0 - lat1 * M_PI / 180.0;
    double dLon = lon2 * M_PI / 180.0 - lon1 * M_PI / 180.0;
    std::cerr << "dLat: " << dLat << "\n";
    std::cerr << "dLon: " << dLon << "\n";
    double latVelocity = dLat * time;
    double lngVelocity = dLon * time;
    std::cerr << "latVel: " << latVelocity << "\n";
    std::cerr << "lngVel: " << lngVelocity << "\n";
    double altVelocity = (alt2 - alt1) * time;
    velocities << latVelocity, lngVelocity, altVelocity;

    return velocities;
}

void GpsImuFusion::gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data)
{
    this->count_imu_topics = 0;
    rclcpp::Time start, end;
    start = this->now();

    double latitude = gps_data->latitude;
    double longitude = gps_data->longitude;
    double altitude = gps_data->altitude;

    Vector3d lla = Vector3d(latitude, longitude, altitude);
    Vector3d velocities;

    /* Check if this is the first Navsat msg */
    if (!is_ref_location_set)
    {
        /* In that case, set reference location */
        this->gndFusion->setRefLocation(latitude, longitude, altitude);
        this->is_ref_location_set = true;
    }

    /* If no IMU data, skip to next iteration */
    if (this->count_imu_topics == 0)
        return;

    RCLCPP_INFO(this->get_logger(), "FUSING GP DATA");

    if (count_gps_topics == 0)
        velocities << 0.1, 0.1, 0;
    else
    {
        double dLat = latitude * M_PI / 180.0 - prev_loc[0] * M_PI / 180.0;
        double dLon = longitude * M_PI / 180.0 - prev_loc[1] * M_PI / 180.0;
        double latVelocity = dLat * 0.1 * 1000000;
        double lngVelocity = dLon * 0.1 * 100000;
        double altVelocity = (altitude - prev_loc[2]) * 0.1;
        velocities << latVelocity, lngVelocity, altVelocity;
        // velocities = measure_distance(prev_loc[0], prev_loc[1], prev_loc[2], latitude, longitude, altitude, 0.1);
    }

    prev_loc = Vector3d(latitude, longitude, altitude);

    this->gndFusion->fusegps(lla, velocities);

    end = this->now();
    rclcpp::Duration exe_time = end - start;

    RCLCPP_INFO(this->get_logger(), "Fusion exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);

    RCLCPP_INFO(this->get_logger(), "Current filter state AFTER FUSION:");
    // this->gndFusion->printCurrentState();

    Vector4d act_orientation;
    Vector3d act_postion;
    Vector3d act_velocities;

    this->gndFusion->pose(act_postion, act_orientation, act_velocities);

    count_gps_topics++;

    if (this->count_gps_topics <= 600)
    {
        // Write to the file
        this->predictFile << "ITER_GPS [" << (count_gps_topics + "") << "]: "
                          << "\n"
                          << "qw: " << act_orientation[0] << "\n"
                          << "qx: " << act_orientation[1] << "\n"
                          << "qy: " << act_orientation[2] << "\n"
                          << "qz: " << act_orientation[3] << "\n"
                          << "pn: " << act_postion[0] << "\n"
                          << "pe: " << act_postion[1] << "\n"
                          << "pd: " << act_postion[2] << "\n";
    }
    else
    {
        // Close the file
        predictFile.close();
    }
}

/* ***** END CALLBACKS ***** */
void GpsImuFusion::updateTf(Vector3d p, geometry_msgs::msg::Quaternion q)
{
    geometry_msgs::msg::TransformStamped t;
    nav_msgs::msg::Odometry odom;

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

    /* Upated Odom header */
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "vehicle";

    /* Updated Odom pose */
    odom.pose.pose.position.x = p[0];
    odom.pose.pose.position.y = p[1];
    odom.pose.pose.position.z = 0.0f;

    /* Update Odom orientation */
    odom.pose.pose.orientation.x = q.x;
    odom.pose.pose.orientation.y = q.y;
    odom.pose.pose.orientation.z = q.z;
    odom.pose.pose.orientation.w = q.w;

    /* Publish new position */
    this->tf_broadcaster_->sendTransform(t);
    this->odom_pub->publish(odom);
}