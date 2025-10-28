#pragma once

#include <Eigen/Dense>

namespace MicrostrainMagCal
{
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);
}