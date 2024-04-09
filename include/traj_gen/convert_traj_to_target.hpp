#ifndef CONVERT_TRAJ_TO_TARGET_HPP
#define CONVERT_TRAJ_TO_TARGET_HPP
#include <math.h>

/**
 * The reduction ratio of LIFT, PAN, Steering, respectively.
*/
const double N_LIFT = 158.0;
const double N_PAN = 3969.0/256.0;
const double N_STEERING = 70.0/40.0;

/**
 * Count per turn Info
*/

const double LIFT_CPT   = 4*4096.0;
const double PAN_CPT    = 4*1024.0;
const double DXL_CPT    = 4096.0;

/**
 * Deg to target value
*/
const double deg2target_LIFT    = LIFT_CPT/360.0 * N_LIFT;
const double deg2target_PAN     = PAN_CPT/360.0 * N_PAN;
const double deg2target_DXL     = DXL_CPT/360.0 * N_STEERING;

/**
 * radpsec to RPM
*/

const double radps2RPM = 60.0/(2.0 * M_PI);

const double l = 0.400;
const double R_wheel = 0.169/2.0;

int32_t convert_deg2target_LIFT(double position);
int32_t convert_deg2target_PAN(double position);
int32_t convert_deg2target_DXL(double position);


#endif