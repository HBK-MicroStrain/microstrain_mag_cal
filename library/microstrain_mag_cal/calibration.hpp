#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        static constexpr uint8_t FIT_OPTIMIZATION_SUCCEEDED                   = 0;
        static constexpr uint8_t FIT_OPTIMIZATION_INSUFFICIENT_INPUT_DATA     = 1;
        static constexpr uint8_t FIT_OPTIMIZATION_DID_NOT_CONVERGE            = 2;
        static constexpr uint8_t FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE  = 3;

        Eigen::Matrix3d soft_iron_matrix;
        Eigen::RowVector3d hard_iron_offset;
        uint8_t error = 0;
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
}
