#pragma once

#include <filesystem>
#include <optional>

#include <mio/mmap.hpp>

#include <microstrain/array_view.hpp>
#include <microstrain/platform.h>  // Needed to fix OS-specific naming conflicts
#include <microstrain_mag_cal/preprocessing.hpp>

namespace backend
{
    struct MappedBinaryData
    {
        mio::mmap_source mapping;
        microstrain::ConstU8ArrayView view;
    };

    std::optional<MappedBinaryData> mapBinaryFile(const std::filesystem::path& filepath);

    microstrain_mag_cal::PointManager extractPointsFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        std::optional<double> reference_field_strength = std::nullopt);

}
