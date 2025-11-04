#pragma once

#include <Eigen/Dense>

// Needed to fix OS-specific naming conflicts
#include <microstrain/platform.h>

namespace Core
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, size_t data_size);
}
