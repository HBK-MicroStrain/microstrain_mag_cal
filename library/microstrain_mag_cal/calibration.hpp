#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        static constexpr uint8_t FIT_OPTIMIZATION_FAILED          = 1;
        static constexpr uint8_t FIT_MATRIX_NOT_POSITIVE_DEFINITE = 2;

        Eigen::Matrix3d soft_iron_matrix;
        Eigen::RowVector3d hard_iron_offset;
        uint8_t error = 0;
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
}
