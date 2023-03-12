#ifndef INS_FILTER_NON_HOLOMIC_STATE
#define INS_FILTER_NON_HOLOMIC_STATE

#include <Quaternion.hpp>
#include <GyroscopeBias.hpp>
#include <NEDPosition.hpp>
#include <NEDVelocities.hpp>
#include <AccelerometerBias.hpp>

class InsFilterNonHolomicState
{
    private:
    /* Definition of the initial states of the filter (16 variables) */

    /* Quaternion defining the orientation of the system */
    Quaternion* q;

    /* Gyroscope Bias */
    GyroscopeBias* gyro_bias;

    /* NED Position */
    NEDPosition* ned_position;

    /* NED Velocities */
    NEDVelocities* ned_velocities;

    /* Accelerometer Bias */
    AccelerometerBias* accel_bias;

    public:

    /* Constructor */
    InsFilterNonHolomicState(Quaternion* q_init, GyroscopeBias* gyro_bias_init, 
        NEDPosition* ned_position_init, NEDVelocities* ned_velocities_init, AccelerometerBias* accel_bias_init);

    /* Destructor */
    ~InsFilterNonHolomicState();

};

#endif /* INS_FILTER_NON_HOLOMIC_STATE */