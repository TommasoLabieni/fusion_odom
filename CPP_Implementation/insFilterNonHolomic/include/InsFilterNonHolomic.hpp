#ifndef INS_FILTER_NON_HOLOMIC
#define INS_FILTER_NON_HOLOMIC

#include <stdint.h>
#include <InsFilterNonHolomicState.hpp>
#include <Eigen/Dense>

using namespace Eigen;
using Eigen::MatrixXd;

class InsFilterNonHolomic
{
    private:

    /* Definition of the initial states of the filter (16 variables) */
    InsFilterNonHolomicState* filter_state;

    /* TODO: SPECIFY PARAMETERS IN A .yaml FILE. And check that you read that correctly */

    /* IMU and GPS frequency */
    uint8_t imu_fs = 100, gps_fs = 10;


    /* Reference frame noise for position and velocity */
    double r_vel = 0, r_pos = 0.005;

    /**
     * The dynamic model of the ground vehicle for this filter assumes there is
     * no side slip or skid during movement. This means that the velocity is 
     * constrained to only the forward body axis. The other two velocity axis 
     * readings are corrected with a zero measurement weighted by the 
     * |zero_velocity_constraint_noise| parameter.
    */ 
    double zero_velocity_constraint_noise = 1e-3;

    /* Sensors noise */
    double gyro_noise = 4.8e-2;
    double gyro_bias_noise = 4e-14;
    double accel_noise = 4.8e-2;
    double accel_bias_noise = 4e-14;

    /** Initial error covariance.
     * This is defined as a 16x16 Identity Matrix with each element in the main diagonal having an 
     * inital value of 1e-e9.
    */
    MatrixXd state_covariance;

    public:
    
    /* Base Constructor  */
    InsFilterNonHolomic(InsFilterNonHolomicState* initial_state);

    /* Destructor */
    ~InsFilterNonHolomic();
};

#endif /* INS_FILTER_NON_HOLOMIC */