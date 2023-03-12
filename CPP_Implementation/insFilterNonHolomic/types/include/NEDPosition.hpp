#ifndef NED_POSITION
#define NED_POSITION

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

#endif /* NED_POSITION */