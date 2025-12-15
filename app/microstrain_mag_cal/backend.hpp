#pragma once

#include <optional>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <microstrain/array_view.hpp>
#include <microstrain/platform.h>  // Needed to fix OS-specific naming conflicts
#include <microstrain_mag_cal/calibration.hpp>

namespace backend
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        std::optional<double> reference_field_strength = std::nullopt);

    nlohmann::json convertFitResultToJson(const microstrain_mag_cal::FitResult &result);

    void writeJsonToFile(const std::filesystem::path &filepath, const nlohmann::json& json_output);
    void writeJsonToFile(const std::filesystem::path &filepath, const microstrain_mag_cal::FitResult& fit_result);
}
