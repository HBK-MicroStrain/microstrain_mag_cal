#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        enum class Error : uint8_t
        {
            NONE,
            FIT_OPTIMIZATION_INSUFFICIENT_INPUT_DATA,
            FIT_OPTIMIZATION_DID_NOT_CONVERGE,
            FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE
        };

        Eigen::Matrix3d soft_iron_matrix;
        Eigen::RowVector3d hard_iron_offset;
        Error error = Error::NONE;
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
}
