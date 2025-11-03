#pragma once

#include <Eigen/Dense>

#include <microstrain/platform.h>

namespace Core
{
    // TODO: Add documentation
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, const size_t data_size);
}
