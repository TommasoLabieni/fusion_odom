#ifndef NED_VELOCITIES
#define NED_VELOCITIES

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

#endif /* NED_VELOCITIES */