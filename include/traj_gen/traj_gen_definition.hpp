#ifndef TRAJ_GEN_DEFINITION_HPP
#define TRAJ_GEN_DEFINITION_HPP

/**
 * The number of LIFT, PAN, WHEEL, and dynamixel, respectively
*/

#define NUM_LIFT        3
#define NUM_PAN         3
#define NUM_WHEEL       3
#define NUM_DXL         3

/**
 * The reduction ratio of LIFT, PAN, Steering, respectively.
*/
const double N_LIFT = 158.0;
const double N_PAN = 3969.0/256.0;
const double N_STEERING = 70.0/40.0;

const double l = 0.400;
const double R_wheel = 0.169/2.0;

#endif