#ifndef CONVERT_TOOLS_HPP
#define CONVERT_TOOLS_HPP
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

const double LIFT_QCPT   = 4*4096.0;
const double PAN_QCPT    = 4*1024.0;
const double DXL_QCPT    = 4096.0;

/**
 * Degree to target value
*/
const double deg2target_LIFT    = LIFT_QCPT/360.0 * N_LIFT;
const double deg2target_PAN     = PAN_QCPT/360.0 * N_PAN;
const double deg2target_DXL     = DXL_QCPT/360.0 * N_STEERING;


/**
 * inc to Degree
*/
const double inc2deg_LIFT   = 360.0 / (LIFT_QCPT*N_LIFT);
const double inc2deg_PAN    = 360.0 / (PAN_QCPT*N_PAN);
const double inc2deg_DXL    = 360.0 / (DXL_QCPT*N_STEERING);

/**
 * Rad per sec to RPM
 * deg per sec to RPM
*/
const double radps2RPM = 60.0/(2.0 * M_PI);
const double degps2RPM = 1.0/6.0;
/**
 * link length of leg and wheel radius
*/
const double l = 0.400;
const double wheel_radius = 0.169/2.0;

int32_t convert_deg2target_LIFT(double position);
int32_t convert_deg2target_PAN(double position);
int32_t convert_deg2target_DXL(double position);

double convert_actual2pos_LIFT(int32_t inc);
double convert_actual2pos_PAN(int32_t inc);
double convert_actual2pos_DXL(int32_t inc);


#endif