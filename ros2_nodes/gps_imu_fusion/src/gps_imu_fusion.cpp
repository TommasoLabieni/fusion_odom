#include <gps_imu_fusion/gps_imu_fusion.hpp>

GpsImuFusion::GpsImuFusion() : rclcpp::Node("gps_imu_fusion_node")
{
    /* Load ROS 2 parameters */
    this->loadParameters();

    /* Create subscribers */
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic, 1, std::bind(&GpsImuFusion::imuDataCallback, this, std::placeholders::_1));

    this->gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->gps_topic, 1, std::bind(&GpsImuFusion::gpsDataCallback, this, std::placeholders::_1));

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

    this->predictFile.open("/tmp/predict_data.txt",std::ios_base::app);

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
    declare_parameter("noises.gps_acc_unc", 0.005);

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
    get_parameter("noises.gps_acc_unc", this->gps_acc_unc);
}

/* ***** CALLBACKS ***** */

void GpsImuFusion::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
{
    this->count_total_topic++;
    rclcpp::Time start, end;
    start = this->now();

    double qx = imu_data->orientation.x;
    double qy = imu_data->orientation.y;
    double qz = imu_data->orientation.z;
    double qw = imu_data->orientation.w;

    if (!this->is_filter_location_initialized)
        return;

    if (!this->is_filter_orientation_initialized)
    {
        this->gndFusion->setInitState(
            Vector4d(qw, qx, qy, qz), /* Initial Quaternion */
            Vector3d(-3.05745876411404e-06, 1.44325973037567e-06, 5.61037701246807e-05),
            Vector3d::Zero(),
            Vector3d::Zero(),
            Vector3d(-7.88350054851016e-06, -1.93254742380793e-07, 3.23255528039867e-06));
        this->is_filter_orientation_initialized = true;
        this->gndFusion->printCurrentState();
        // RCLCPP_INFO(this->get_logger(), "Inital state covariance:");
        // this->gndFusion->printCurrentStateCovariance();
    }

    /* Convert IMU accel_data to eigen vector */
    Vector3d accel_data(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);

    Vector3d gyro_data(imu_data->angular_velocity.x, imu_data->angular_velocity.y, -imu_data->angular_velocity.z);

    // RCLCPP_INFO(this->get_logger(), "PREDICTING NEW SYSTEM STATE");

    /* If vehicle is still don't updated filter state */
    if(!this->is_vehicle_still)
    {
        /* Predict */
        this->gndFusion->predict(accel_data, gyro_data);
    }
    end = this->now();
    rclcpp::Duration exe_time = end - start;

    // RCLCPP_INFO(this->get_logger(), "Predict exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);

    // RCLCPP_INFO(this->get_logger(), "Current filter state AFTER PREDICTION:");
    // this->gndFusion->printCurrentState();

    Vector4d act_orientation;
    Vector3d act_postion;
    Vector3d act_velocities;

    this->gndFusion->pose(act_postion, act_orientation, act_velocities);

    this->count_imu_topics++;
    /* Update vehicle TF */
    tf2::Quaternion q(act_orientation[1], act_orientation[2], act_orientation[3], act_orientation[0]);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw *= (-M_PI / 3);
    q.setRPY(roll, pitch, yaw);
    
    // RCLCPP_INFO(this->get_logger(), "Publishing Tf and Odometry");
    this->updateTf(act_postion, q);
}

void GpsImuFusion::gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data)
{
    this->count_total_topic++;
    count_imu_topics = 0;
    rclcpp::Time start, end;
    start = this->now();

    double latitude = gps_data->latitude;
    double longitude = gps_data->longitude;
    double altitude = gps_data->altitude;

    Vector3d lla = Vector3d(latitude, longitude, altitude);
    Vector3d velocities;

    double time = 1.0 / this->gps_fs;
    double R = 6378.137; 
    double dLat = latitude * M_PI / 180.0 - prev_loc[0] * M_PI / 180.0;
    double dLon = longitude * M_PI / 180.0 - prev_loc[1] * M_PI / 180.0;
    double a = sin(dLat/2.0) * sin(dLat/2.0) + cos(prev_loc[0] * M_PI / 180) * cos(latitude * M_PI / 180.0) * sin(dLon/2.0) * sin(dLon/2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    double d = R * c;
    this->total_dist += (d*1000);
    this->act_dist += (d*1000);
    /* Update previous location */
    prev_loc = Vector3d(latitude, longitude, altitude);
    d *= 1000.0; /* Distance in meters */
    if ((d*100) < (this->gps_acc_unc*100))
    {
        RCLCPP_INFO(this->get_logger(), "Vehicle is still. Skipping.");
        this->is_vehicle_still = true;
        return;
    } else
    {
        this->is_vehicle_still = false;
        if ((this->count_gps_topics % 10) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "You are traveling at %lf m/s == %lf km/h", this->act_dist, (this->act_dist * 3.6));
            this->act_dist = 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Vehicle is moving of %lf", d*100);
    }
    double latVelocity = d * time;
    double lngVelocity = d * time;
    velocities << dLat, dLon, 0.0;
    // std::cerr << "Vx : " << velocities[0] << " Vy: " << velocities[1] << "\n";
    // velocities = measure_distance(prev_loc[0], prev_loc[1], prev_loc[2], latitude, longitude, altitude, 0.1);


    /* Check if this is the first Navsat msg */
    if (!is_filter_location_initialized && !this->is_vehicle_still)
    {
        /* In that case, set reference location */
        this->gndFusion->setRefLocation(latitude, longitude, altitude);
        this->is_filter_location_initialized = true;
        velocities << 3.29344e-08, 3.09796e-08, 0;
    } 

    if (!this->is_filter_orientation_initialized)
        return;
    // RCLCPP_INFO(this->get_logger(), "FUSING GP DATA");

    this->gndFusion->fusegps(lla, velocities);

    end = this->now();
    rclcpp::Duration exe_time = end - start;

    // RCLCPP_INFO(this->get_logger(), "Fusion exe time (ms): %f", (float)exe_time.nanoseconds() / 1e6);

    // RCLCPP_INFO(this->get_logger(), "Current filter state AFTER FUSION:");
    // this->gndFusion->printCurrentState();
    count_gps_topics++;
}

/* ***** END CALLBACKS ***** */
void GpsImuFusion::updateTf(Vector3d p, tf2::Quaternion q)
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
    t.transform.rotation.x = q.getX();
    t.transform.rotation.y = q.getY();
    t.transform.rotation.z = q.getZ();
    t.transform.rotation.w = q.getW();

    /* Upated Odom header */
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "vehicle";

    /* Updated Odom pose */
    odom.pose.pose.position.x = p[0];
    odom.pose.pose.position.y = p[1];
    odom.pose.pose.position.z = 0.0f;

    /* Update Odom orientation */
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    odom.pose.pose.orientation.w = q.getW();

    /* Publish new position */
    this->tf_broadcaster_->sendTransform(t);
    this->odom_pub->publish(odom);
}
