#include <InsFilterNonHolomic.hpp>

InsFilterNonHolomic::InsFilterNonHolomic(InsFilterNonHolomicState* initial_state)
{
    this->filter_state = initial_state;
    
    /* create initial error covariance */
    this->state_covariance = Matrix<double, 16, 16>::Identity();
}

InsFilterNonHolomic::~InsFilterNonHolomic()
{
    ;
}