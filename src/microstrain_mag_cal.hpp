#pragma once

#include <Eigen/Dense>

namespace MicrostrainMagCal
{
    // TODO: Document Nx3 matrix constructor limitation when creating an empty matrix.
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);
}