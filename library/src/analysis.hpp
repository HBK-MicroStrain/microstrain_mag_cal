#pragma once

#include <calibration.hpp>


namespace microstrain_mag_cal
{
    // ---------------------------------------------------------------------------------------------
    // Data
    // ---------------------------------------------------------------------------------------------

    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    // ---------------------------------------------------------------------------------------------
    // Fit
    // ---------------------------------------------------------------------------------------------

    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, double field_strength);
}
