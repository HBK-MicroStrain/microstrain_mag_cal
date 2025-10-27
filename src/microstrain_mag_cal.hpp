#pragma once

#include <Eigen/Dense>

namespace MicrostrainMagCal
{
    // Input: Nx3 Matrix
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);
}