#include "convert_tools.hpp"

int32_t convert_deg2target_LIFT(double position)
{
    return position*deg2target_LIFT;
}

int32_t convert_deg2target_PAN(double position)
{
    return position*deg2target_PAN;
}

int32_t convert_deg2target_DXL(double position)
{
    return position*deg2target_DXL + 2048;
}

double convert_actual2pos_LIFT(int32_t inc)
{
    return inc*inc2deg_LIFT;
}

double convert_actual2pos_PAN(int32_t inc)
{
    return inc*inc2deg_PAN;
}

double convert_actual2pos_DXL(int32_t inc)
{
    return inc*inc2deg_DXL;
}