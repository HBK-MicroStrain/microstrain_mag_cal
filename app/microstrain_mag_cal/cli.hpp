#pragma once

#include <string>

#include <CLI/CLI.hpp>

#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_mag_cal/preprocessing.hpp>

namespace cli
{
    struct ProgramArgs
    {
        explicit ProgramArgs(char** argv);

        CLI::App app{};

        std::filesystem::path input_data_filepath;
        bool display_analysis = false;
        bool spherical_fit = false;
        bool ellipsoidal_fit = false;
        std::optional<double> field_strength;
        std::filesystem::path output_json_directory;
    };

    void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, double fit_RMSE);
    std::string getPointUsageDisplay(const microstrain_mag_cal::PointManager &point_manager);
}
