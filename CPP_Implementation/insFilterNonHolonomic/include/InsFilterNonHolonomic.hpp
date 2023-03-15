#pragma once

#include <stdint.h>
#include <gps.h>
#include <InsFilterNonHolonomicState.hpp>
#include <InsFilterNonHolonomicTypes.hpp>
#include <Eigen/Dense>


using namespace Eigen;
using Eigen::MatrixXd;

class InsFilterNonHolonomic
{
private:

    /* IMU and GPS frequency */
    uint8_t imu_fs = 100, gps_fs = 10;

    /* Initial system location [latitude longitude altitude]*/
    gps_fix_t reference_location;

    /* Decimation Factor */
    uint8_t decimation_factor = 1;

    /* Definition of the initial states of the filter (16 variables) */
    InsFilterNonHolonomicState* filter_state_old;
    MatrixXd filter_state;

    /** Initial error covariance.
     * This is defined as a 16x16 Identity Matrix with each element in the main diagonal having an 
     * inital value of 1e-e9.
    */
    MatrixXd state_covariance;

    /* Define sensors noise */
    Vector3f gyroscope_noise;                   /* Gyro noise */
    Vector3f gyroscope_bias_noise;              /* Gyro bias noise */
    float gyroscope_bias_decay_factor = 1.0f;   /* A decay factor of 0 models gyroscope bias as a white noise process. A decay factor of 1 models the gyroscope bias as a random walk process */
    Vector3f accelerometer_noise;               /* Accel noise */
    Vector3f accelerometer_bias_noise;          /* Accel bias noise */
    float accel_bias_decay_factor = 1.0f;       /* A decay factor of 0 models gyroscope bias as a white noise process. A decay factor of 1 models the gyroscope bias as a random walk process */

    /**
     * The dynamic model of the ground vehicle for this filter assumes there is
     * no side slip or skid during movement. This means that the velocity is 
     * constrained to only the forward body axis. The other two velocity axis 
     * readings are corrected with a zero measurement weighted by the 
     * |zero_velocity_constraint_noise| parameter.
    */ 
    float zero_velocity_constraint_noise = 1e-3;/* Velocity constraints noise in (m/s)^2 */

    /* TODO: SPECIFY PARAMETERS IN A .yaml FILE. And check that you read that correctly */

    /**
     * Predict forward the state estimate one time sample, based on current IMU data
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
    */
    void setState(Vector3f accel_data, Vector3f gyro_data);

    /**
     * Jacobian of process equations. Compute the Jacobian matrix F 
     * of the state transition function f(x) with respect to state x.
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
    */
    MatrixXd stateTransitionJacobianFcn(Vector3f accel_data, Vector3f gyro_data);

public:
    
    /* Base Constructor  */
    InsFilterNonHolonomic(InsFilterNonHolonomicState* initial_state);

    /**
     *  Update Filter state by prediction, using gyroscope AND accelerometer data.
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
    */
    void predict(Vector3f accel_data, Vector3f gyro_data);


    /**
     * Correct filter state using GPS data.
     * 
     * @param[in] lla_position Position of GPS receiver in geodetic latitude, longitude, and altitude (LLA) format. Latitude and longitude are in degrees with north and east being positive. Altitude is in meters.
     * @param[in] position_covariance Position measurement covariance of GPS receiver in m^2.
     * @param[in] velocity Velocity of the GPS receiver in the local NED coordinate system in m/s
     * @param[in] velocity_covariance measurement covariance of the GPS receiver in the local NED coordinate system in (m/s)^2
     * @param[out] out Position and course residual, returned as a 1-by-6 vector of real values in m and rad/s, respectively.
     * @param[out] residual_covariance Residual covariance
    */
    void correct(Vector3f lla_position, Matrix3f position_covariance, Vector3f velocity, Matrix3f velocity_covariance, 
        MatrixXf &out, MatrixXf &residual_covariance);

    /**
     * Return current system position, orientation and velocity
     * 
     * @param[out] curr_position Position estimate expressed in the local coordinate system of the filter in meters
     * @param[out] curr_orientation Orientation estimate expressed in the local coordinate system of the filter 
     * @param[out] curr_velocity Velocity estimate expressed in the local coordinate system of the filter in m/s
    */
    void pose(InsFilterNonHolonomicTypes::NEDPosition &curr_position, InsFilterNonHolonomicTypes::Quaternion &curr_orientation, 
    InsFilterNonHolonomicTypes::NEDVelocities &curr_velocity);

    /**
     * Returns current filter state
    */
    InsFilterNonHolonomicState getCurrentState();

    /* Destructor */
    ~InsFilterNonHolonomic();
};