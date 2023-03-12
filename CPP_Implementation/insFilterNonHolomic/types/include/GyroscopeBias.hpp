#ifndef GYROSCOPE_BIAS
#define GYROSCOPE_BIAS

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

#endif /* GYROSCOPE_BIAS */