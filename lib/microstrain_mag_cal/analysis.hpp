#pragma once

#include <microstrain_mag_cal/calibration.hpp>


namespace microstrain_mag_cal
{
    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::Vector3d &initial_offset);

    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, double field_strength);
}
