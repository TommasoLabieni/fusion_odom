#ifndef QUATERNION
#define QUATERNION

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

#endif /* QUATERNION */