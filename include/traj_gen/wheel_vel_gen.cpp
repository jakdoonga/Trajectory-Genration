#include "wheel_vel_gen.hpp"
#include <iostream>

WHEEL_VEL_GEN::WHEEL_VEL_GEN()
{

}

void WHEEL_VEL_GEN::get_wheel_vel(
    double* wheel_linear_vel,
    double* wheel_angular_vel,
    double* pos_sensor,
    double* des_vel,
    WHEEL_VEL_MODE wheel_vel_mode
)
{
    switch(wheel_vel_mode)
    {
        case PAN_WHEEL:
            for(int i = 0; i < 3; i++)
            {
                wheel_linear_vel[i] = 
                l*des_vel[i]*sin(pos_sensor[i+3]*M_PI/180.0);
                
                wheel_angular_vel[i] = 
                wheel_linear_vel[i]/wheel_radius;
            }
            break;

        case LIFT_WHEEL:
            for(int i = 0; i < 3; i++)
            {
                wheel_linear_vel[i] =
                l*des_vel[i+3]*cos(pos_sensor[i+3]*M_PI/180.0);
                
                wheel_angular_vel[i] = 
                wheel_linear_vel[i]/wheel_radius;
            }
            break;
        default:
            break;

    }

}


