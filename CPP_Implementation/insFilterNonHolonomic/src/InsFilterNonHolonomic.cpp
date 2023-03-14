#include <InsFilterNonHolonomic.hpp>

InsFilterNonHolonomic::InsFilterNonHolonomic(InsFilterNonHolonomicState* initial_state)
{
    this->filter_state_old = initial_state;
    
    this->filter_state = Matrix<double, 16, 1>();

    /* create initial error covariance */
    this->state_covariance = Matrix<double, 16, 16>::Identity();
}

void InsFilterNonHolonomic::setState(Vector3f accel_data, Vector3f gyro_data)
{
    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    double gbX = this->filter_state(4, 0);
    double gbY = this->filter_state(5, 0);
    double gbZ = this->filter_state(6, 0);
    double pn = this->filter_state(7, 0);
    double pe = this->filter_state(8, 0);
    double pd = this->filter_state(9, 0);
    double vn = this->filter_state(10, 0);
    double ve = this->filter_state(11, 0);
    double vd = this->filter_state(12, 0);
    double abX = this->filter_state(13, 0);
    double abY = this->filter_state(14, 0);
    double abZ = this->filter_state(15, 0);

    /* Retrieve sensors data */
    double amX = accel_data(0);
    double amY = accel_data(1);
    double amZ = accel_data(2);
    double gmX = gyro_data(0);
    double gmY = gyro_data(1);
    double gmZ = gyro_data(2);

    double dt = 1 / this->imu_fs;

    double lambda_gyro = 1 - this->gyroscope_bias_decay_factor;
    double lambda_accel = 1 - this->accel_bias_decay_factor;

    /* NED system -> Gravity is on the Z axis */
    double gravX = 0.0f;
    double gravY = 0.0f;
    double gravZ = -9.81f;

    /* Set new system state. To understand the following code read insFilterNonholonomic documentation */
    
    /* New predicted orientation */
    this->filter_state(0, 0) = q0 + dt*q1*(gbX/2 - gmX/2) + dt*q2*(gbY/2 - gmY/2) + dt*q3*(gbZ/2 - gmZ/2);  //q0
    this->filter_state(1, 0) = q1 - dt*q0*(gbX/2 - gmX/2) + dt*q3*(gbY/2 - gmY/2) - dt*q2*(gbZ/2 - gmZ/2);  //q1
    this->filter_state(2, 0) = q2 - dt*q3*(gbX/2 - gmX/2) - dt*q0*(gbY/2 - gmY/2) + dt*q1*(gbZ/2 - gmZ/2);  //q2
    this->filter_state(3, 0) = q3 + dt*q2*(gbX/2 - gmX/2) - dt*q1*(gbY/2 - gmY/2) - dt*q0*(gbZ/2 - gmZ/2);  //q3
    
    /* New predicted gyro_bias */
    this->filter_state(4, 0) = -gbX*(dt*lambda_gyro - 1);  //gyro_bias_x
    this->filter_state(5, 0) = -gbY*(dt*lambda_gyro - 1);  //gyro_bias_y
    this->filter_state(6, 0) = -gbZ*(dt*lambda_gyro - 1);  //gyro_bias_z

    /* New predicted position */
    this->filter_state(7, 0) = pn + dt*vn;  //pos_N
    this->filter_state(8, 0) = pe + dt*ve;  //pos_E
    this->filter_state(9, 0) = pd + dt*vd;  //pos_D

    /* New predicted velocies */
    this->filter_state(10, 0) = vn + dt*(q0*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)) - gravX + q2*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) + q1*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)) - q3*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)));  //vel_N
    this->filter_state(11, 0) = ve + dt*(q0*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)) - gravY - q1*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) + q2*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)) + q3*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)));  //vel_E
    this->filter_state(12, 0) = vd + dt*(q0*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) - gravZ + q1*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)) - q2*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)) + q3*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)));

    /* New predicted accel_bias */
    this->filter_state(13, 0) = -abX*(dt*lambda_accel - 1);  //accel_bias_x
    this->filter_state(14, 0) = -abY*(dt*lambda_accel - 1);  //accel_bias_y
    this->filter_state(15, 0) = -abZ*(dt*lambda_accel - 1);  //accel_bias_z

    InsFilterNonHolonomicTypes::Quaternion::repair({
        this->filter_state(0, 0),
        this->filter_state(1, 0),
        this->filter_state(2, 0),
        this->filter_state(3, 0),
    });

}

MatrixXd InsFilterNonHolonomic::stateTransitionJacobianFcn(Vector3f accel_data, Vector3f gyro_data)
{
    MatrixXd F = Matrix<double, 16, 16>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    double gbX = this->filter_state(4, 0);
    double gbY = this->filter_state(5, 0);
    double gbZ = this->filter_state(6, 0);
    double pn = this->filter_state(7, 0);
    double pe = this->filter_state(8, 0);
    double pd = this->filter_state(9, 0);
    double vn = this->filter_state(10, 0);
    double ve = this->filter_state(11, 0);
    double vd = this->filter_state(12, 0);
    double abX = this->filter_state(13, 0);
    double abY = this->filter_state(14, 0);
    double abZ = this->filter_state(15, 0);

    double dt = 1 / this->imu_fs;

    /* Retrieve sensors data */
    double amX = accel_data(0);
    double amY = accel_data(1);
    double amZ = accel_data(2);
    double gmX = gyro_data(0);
    double gmY = gyro_data(1);
    double gmZ = gyro_data(2);

    double lambda_gyro = 1 - this->gyroscope_bias_decay_factor;
    double lambda_accel = 1 - this->accel_bias_decay_factor;

    F <<
        1,                                                            dt*(gbX/2 - gmX/2),                                           dt*(gbY/2 - gmY/2),                                           dt*(gbZ/2 - gmZ/2),         (dt*q1)/2,         (dt*q2)/2,         (dt*q3)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbX/2 - gmX/2),                                          1,                                          -dt*(gbZ/2 - gmZ/2),                                           dt*(gbY/2 - gmY/2),        -(dt*q0)/2,         (dt*q3)/2,        -(dt*q2)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbY/2 - gmY/2),                                           dt*(gbZ/2 - gmZ/2),                                                            1,                                          -dt*(gbX/2 - gmX/2),        -(dt*q3)/2,        -(dt*q0)/2,         (dt*q1)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbZ/2 - gmZ/2),                                          -dt*(gbY/2 - gmY/2),                                           dt*(gbX/2 - gmX/2),                                                            1,         (dt*q2)/2,        -(dt*q1)/2,        -(dt*q0)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0, 1 - dt*lambda_gyro,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0, 1 - dt*lambda_gyro,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0, 1 - dt*lambda_gyro, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 1, 0, 0, dt,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 1, 0,  0, dt,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 1,  0,  0, dt,                              0,                              0,                              0,
        dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),  dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)), -dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  1,  0,  0, dt*(pow(q0,2) + pow(q1,2) - pow(q2,2) - pow(q3,2)),        -dt*(2*q0*q3 - 2*q1*q2),         dt*(2*q0*q2 + 2*q1*q3),
        dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)), -dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),  dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  0,  1,  0,         dt*(2*q0*q3 + 2*q1*q2), dt*(pow(q0,2) - pow(q1,2) + pow(q2,2) - pow(q3,2)),        -dt*(2*q0*q1 - 2*q2*q3),
        dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)),  dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)), -dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  0,  0,  1,        -dt*(2*q0*q2 - 2*q1*q3),         dt*(2*q0*q1 + 2*q2*q3), dt*(pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2)),
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,             1 - dt*lambda_accel,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,             1 - dt*lambda_accel,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,             1 - dt*lambda_accel;
    ;

    return F;
}
void InsFilterNonHolonomic::predict(Vector3f accel_data, Vector3f gyro_data)
{
    this->setState(accel_data, gyro_data);
    MatrixXd F = this->stateTransitionJacobianFcn(accel_data, gyro_data);
    /* TODO: Calc G, U, new Covariance. FROM line 162 of NHConstrainedIMUGOSFuserBase.m */
}


void InsFilterNonHolonomic::correct(
    Vector3f lla_position,
    Matrix3f position_covariance,
    Vector3f velocity,
    Matrix3f velocity_covariance, 
    MatrixXf &out,
    MatrixXf &residual_covariance)
{
    ;
}


void InsFilterNonHolonomic::pose(InsFilterNonHolonomicTypes::NEDPosition &curr_position, InsFilterNonHolonomicTypes::Quaternion &curr_orientation, 
    InsFilterNonHolonomicTypes::NEDVelocities &curr_velocity)
{
    /* Retrieve actual filter state */
    InsFilterNonHolonomicState act_state = this->getCurrentState();

    /* Set actual position */
    curr_position = act_state.getActualPosition();

    /* TODO: Set actual orientation */
    // curr_orientation = InsFilterNonHolonomicTypes::Quaternion(x,x,x,x);

    /* Set actual velocities */
    curr_velocity = act_state.getActualVelocities();
}

InsFilterNonHolonomicState InsFilterNonHolonomic::getCurrentState()
{
    return *(this->filter_state_old);
}

InsFilterNonHolonomic::~InsFilterNonHolonomic()
{
    ;
}