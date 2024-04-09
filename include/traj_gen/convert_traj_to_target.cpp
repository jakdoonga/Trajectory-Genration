#include "convert_traj_to_target.hpp"

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