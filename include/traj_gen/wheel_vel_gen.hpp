#ifndef WHEEL_VEL_GEN_HPP
#define WHEEL_VEL_GEN_HPP
#include "convert_tools.hpp"
#include <math.h>

/**
 * Wheel velocity mode
*/
enum WHEEL_VEL_MODE{
    PAN_WHEEL,
    LIFT_WHEEL
};

class WHEEL_VEL_GEN{

    public:

        WHEEL_VEL_GEN();

        /**
         * pos_sensor[0] ~ pos_sensor[2]: position of PAN [deg]
         * pos_sensor[3] ~ pos_sensor[5]: position of LIFT [deg]
         * 
         * des_vel[0] ~ des_vel[2]: desired velocity of PAN [deg/s]
         * des_vel[3] ~ des_vel[5]: desired velocity of LIFT [deg/s]
        */

        void get_wheel_vel(
            double* wheel_linear_vel,
            double* wheel_angular_vel,
            double* pos_sensor,
            double* des_vel,
            WHEEL_VEL_MODE wheel_vel_mode
        );


    private:

    

};




#endif