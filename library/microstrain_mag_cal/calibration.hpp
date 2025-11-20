#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        FitResult(Eigen::Matrix3d soft_iron_matrix, Eigen::Vector3d hard_iron_offset, const bool succeeded)
            : soft_iron_matrix(std::move(soft_iron_matrix)),
              hard_iron_offset(std::move(hard_iron_offset)),
              succeeded(succeeded) {}

        const Eigen::Matrix3d soft_iron_matrix;
        const Eigen::RowVector3d hard_iron_offset;
        const bool succeeded;
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
}
