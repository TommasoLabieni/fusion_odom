#pragma once

namespace InsFilterNonHolonomicTypes
{
    /**
     *  Class that defines the bias of an Accelerometer 
    */

    class AccelerometerBias
    {
        private:
        double accel_bias_x, accel_bias_y, accel_bias_z;

        public:
        
        /* Constructor */
        AccelerometerBias(double accel_bias_x, double accel_bias_y, double accel_bias_z);

        /* Destructor */
        ~AccelerometerBias();
    };

    /**
     *  Class that defines the bias of a Gyroscope 
    */

    class GyroscopeBias
    {
        private:
        double gyro_bias_x, gyro_bias_y, gyro_bias_z;

        public:
        
        /* Constructor */
        GyroscopeBias(double gyro_bias_x, double gyro_bias_y, double gyro_bias_z);

        /* Destructor */
        ~GyroscopeBias();
    };

    /**
     *  Class that defines the position of the system using North-East-Down (NED) Reference Frame
    */

    class NEDPosition
    {
        private:
        double pos_N, pos_E, pos_D;

        public:
        
        /* Constructor */
        NEDPosition(double pos_N, double pos_E, double pos_D);

        /* Destructor */
        ~NEDPosition();
    };

    /**
     *  Class that defines the velocities of the system using North-East-Down (NED) Reference Frame
    */

    class NEDVelocities
    {
        private:
        double vel_N, vel_E, vel_D;

        public:
        
        /* Constructor */
        NEDVelocities(double vel_N, double vel_E, double vel_D);

        /* Destructor */
        ~NEDVelocities();
    };

    /**
     *  Class that defines a quaternion.
     * 
     * A quaternion is a four-part hyper-complex number used in three-dimensional rotations and orientations.
     * A quaternion number is represented in the form q0 + q1i + q2j + q3k, 
     * where a0, q1, q2, and q3 parts are real numbers, and i, j, and k are the basis elements, 
     * satisfying the equation: i2 = j2 = k2 = ijk = −1.
    */
    class Quaternion
    {
        private:
        double q0, q1, q2, q3;

        public:
        
        /* Constructor */
        Quaternion(double q0, double q1, double q2, double q3);

        /* Destructor */
        ~Quaternion();
    };

}