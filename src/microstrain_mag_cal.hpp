#pragma once

//#include <utility>

#include <Eigen/Dense>


namespace MicrostrainMagCal
{
    struct FitResult
    {
        enum class Status { None, Invalid, Valid };

        FitResult(
            Eigen::Matrix<double, 3, 3> soft_iron_matrix,
            Eigen::Vector3d hard_iron_offset,
            const Status status)
            : soft_iron_matrix(std::move(soft_iron_matrix)),
              hard_iron_offset(std::move(hard_iron_offset)),
              status(status) {}

        const Eigen::Matrix<double, 3, 3> soft_iron_matrix;
        const Eigen::Vector3d hard_iron_offset;
        const Status status;
    };

    // TODO: Document Nx3 matrix constructor limitation when creating an empty matrix.
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);
    // TODO: Document using Geography convention over Physics convention.
    double calculate_spatial_coverage(const Eigen::MatrixX3d &points);
    FitResult calculate_spherical_fit(const Eigen::MatrixX3d &points, double field_strength);
}