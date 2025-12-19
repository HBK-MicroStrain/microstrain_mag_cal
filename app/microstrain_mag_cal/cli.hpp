#pragma once

#include <string>

#include <microstrain_mag_cal/calibration.hpp>

namespace cli
{
    void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, double fit_RMSE);
}
