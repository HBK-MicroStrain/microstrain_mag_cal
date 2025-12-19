#pragma once

#include <string>

#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_mag_cal/preprocessing.hpp>

namespace cli
{
    void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, double fit_RMSE);

    std::string getPointUsageDisplay(const microstrain_mag_cal::PointManager &point_manager);
}
