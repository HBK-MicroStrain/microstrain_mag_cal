#pragma once

#include <Eigen/Dense>

namespace MicrostrainMagCal
{
    // TODO: Document Nx3 matrix constructor limitation when creating an empty matrix.
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);
    // TODO: Document using Geography convention over Physics convention.
    double calculate_spatial_coverage(const Eigen::MatrixX3d &points);
}