#include <InsFilterNonHolonomic.hpp>
#include <InsFilterNonHolonomicState.hpp>
#include <InsFilterNonHolonomicTypes.hpp>

int main(void)
{
    InsFilterNonHolonomic* gndFilter;
    Vector4d q_init; 
    Vector3d gyro_bias_init; 
    Vector3d position_init; 
    Vector3d velocity_init; 
    Vector3d accel_bias_init;

    q_init << 1.0, 0.0, 0.0, 0.0;
    gyro_bias_init << 0.000265955156806132, 0.000118613066929290, -0.000865860620781665;
    position_init << 0.0, 0.0, 0.0;
    velocity_init << 0.0, 0.0, 0.0;
    accel_bias_init << 9.42558485122576e-05, -2.35931658257626e-05, 0.000160709506499922;

    gndFilter = new InsFilterNonHolonomic();
    gndFilter->setInitState(q_init, gyro_bias_init, position_init, velocity_init, accel_bias_init);
    gndFilter->printCurrentState();

    delete gndFilter;
}