#include <InsFilterNonHolonomic.hpp>

InsFilterNonHolonomic::InsFilterNonHolonomic(InsFilterNonHolonomicState* initial_state)
{
    this->filter_state = initial_state;
    
    /* create initial error covariance */
    this->state_covariance = Matrix<double, 16, 16>::Identity();
}

void InsFilterNonHolonomic::predict(Vector3f accel_data, Vector3f gyro_data)
{
    ;
}


void InsFilterNonHolonomic::correct(Vector3f lla_position, Matrix3f position_covariance, Vector3f velocity, Matrix3f velocity_covariance, 
        MatrixXf &out, MatrixXf &residual_covariance)
{
    ;
}


InsFilterNonHolonomicState InsFilterNonHolonomic::getCurrentState()
{
    return *(this->filter_state);
}

InsFilterNonHolonomic::~InsFilterNonHolonomic()
{
    ;
}