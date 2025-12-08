#pragma once

#include <mag_cal_core/calibration.hpp>


namespace mag_cal_core
{
    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, double field_strength);
}
