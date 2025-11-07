#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    // ---------------------------------------------------------------------------------------------
    // Data Structures
    // ---------------------------------------------------------------------------------------------

    struct FitResult
    {
        FitResult(Eigen::Matrix<double, 3, 3> soft_iron_matrix, Eigen::Vector3d hard_iron_offset, const bool succeeded) :
            soft_iron_matrix(std::move(soft_iron_matrix)),
            hard_iron_offset(std::move(hard_iron_offset)),
            succeeded(succeeded) {}

        const Eigen::Matrix<double, 3, 3> soft_iron_matrix;
        const Eigen::Vector3d hard_iron_offset;
        const bool succeeded;
    };

    // ---------------------------------------------------------------------------------------------
    // Initial Parameter Estimation
    // ---------------------------------------------------------------------------------------------

    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);

    // ---------------------------------------------------------------------------------------------
    // Data Statistics
    // ---------------------------------------------------------------------------------------------

    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);
    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    // ---------------------------------------------------------------------------------------------
    // Calibration Fitting
    // ---------------------------------------------------------------------------------------------

    FitResult calculateSphericalFit(
        const Eigen::MatrixX3d &points,
        double field_strength,
        const Eigen::RowVector3d &initial_offset);

    FitResult calculateEllipsoidalFit(
        const Eigen::MatrixX3d &points,
        double field_strength,
        const Eigen::RowVector3d &initial_offset);

    // ---------------------------------------------------------------------------------------------
    // Fit Quality metrics
    // ---------------------------------------------------------------------------------------------

    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, double field_strength);
}