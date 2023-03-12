#ifndef ACCELEROMETER_BIAS
#define ACCELEROMETER_BIAS

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

#endif /* ACCELEROMETER_BIAS */