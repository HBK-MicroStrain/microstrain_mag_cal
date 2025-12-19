#pragma once

#include <optional>

#include <microstrain/array_view.hpp>
#include <microstrain/platform.h>  // Needed to fix OS-specific naming conflicts
#include <microstrain_mag_cal/preprocessing.hpp>

namespace backend
{
    microstrain_mag_cal::PointManager extractPointsFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        std::optional<double> reference_field_strength = std::nullopt);
}
