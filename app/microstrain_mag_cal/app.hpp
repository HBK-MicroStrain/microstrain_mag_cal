#pragma once

#include <optional>

#include <Eigen/Dense>

#include "microstrain/array_view.hpp"
#include <microstrain/platform.h>  // Needed to fix OS-specific naming conflicts


namespace app
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        std::optional<double> reference_field_strength = std::nullopt);
}
