#pragma once

#include <microstrain_mag_cal/calibration.hpp>


namespace microstrain_mag_cal
{
    FitResult composeCorrections(const FitResult &old_fit, const FitResult &new_fit);
};
