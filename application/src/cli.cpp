#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <mio/mmap.hpp>

#include <mag_cal_core.hpp>
#include "microstrain_mag_cal/analysis.hpp"
#include "microstrain_mag_cal/calibration.hpp"


// Adds custom formatting for --help output.
//
// All settings that aren't overridden will retain their defaults.
//
class HelpMessageFormatter final : public CLI::Formatter
{
public:
    // Output for usage line
    std::string make_usage(const CLI::App* app, const std::string name) const override
    {
       return "USAGE: " + name + (app->get_help_ptr() != nullptr ? " [OPTIONS]" : "") + "\n";
    }
};

std::string getErrorMessage(const uint8_t error)
{
    const std::string error_code = "(" + std::to_string(error) + ") ";

    switch (error)
    {
        case microstrain_mag_cal::FitResult::FIT_OPTIMIZATION_FAILED: return error_code + "FIT OPTIMIZATION FAILED";
        case microstrain_mag_cal::FitResult::FIT_MATRIX_NOT_POSITIVE_DEFINITE: return error_code + "FIT MATRIX NOT POSITIVE DEFINITE";
        default: return error_code + "UNKNOWN ERROR";
    }
}

// Console output after the fitting algorithms are run
void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, const double fit_RMSE)
{
    static constexpr int MIN_SUPPORTED_TERMINAL_WIDTH = 50;

    printf("%s\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());
    printf("%s\n", fit_name.data());
    printf("%s\n\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());

    printf("Fit Result: ");
    if (result.error)
    {
        printf("FAILED ---> Error: %s\n\n", getErrorMessage(result.error).c_str());
    }
    else
    {
        printf("SUCCEEDED\n\n");
    }

    printf("Soft-Iron Matrix:\n");
    std::cout << result.soft_iron_matrix << "\n\n";

    printf("Hard-Iron Offset:\n");
    std::cout << result.hard_iron_offset << "\n\n";

    printf("Fit RMSE: %.5f\n\n", fit_RMSE);
}


int main(const int argc, char **argv)
{
    /*** Parse commandline arguments ***/

    std::filesystem::path arg_filepath;
    std::optional<double> arg_field_strength;
    bool arg_spatial_coverage = false;
    bool arg_spherical_fit = false;
    bool arg_ellipsoidal_fit = false;

    CLI::App app{"MVP converting the mag cal logic from InertialConnect into a standalone application."};
    app.formatter(std::make_shared<HelpMessageFormatter>());

    app.add_option("file", arg_filepath, "A binary file containing mip data to read from.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_option("-r,--reference-field-strength", arg_field_strength, "Field strength to use as a reference instead of using the measured field strength.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-c,--spatial-coverage", arg_spatial_coverage, "Calculate the spatial coverage of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-s,--spherical-fit", arg_spherical_fit, "Perform a spherical fit on the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-e,--ellipsoidal-fit", arg_ellipsoidal_fit, "Perform an ellipsoidal fit on the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    CLI11_PARSE(app, argc, argv);

    /*** Create a read-only view of the input file ***/

    std::error_code error;
    const mio::mmap_source file_view = mio::make_mmap_source(arg_filepath.string(), error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_view.data());
    const microstrain::ConstU8ArrayView data_view(data, file_view.size());

    /*** Run the calculations ***/

    const Eigen::MatrixX3d points = mag_cal_core::extractPointMatrixFromRawData(data_view, arg_field_strength);
    const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

    printf("Number Of Points: %lld\n\n", points.rows());

    if (!arg_field_strength.has_value())
    {
        arg_field_strength = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(points, initial_offset);
    }

    if (arg_spherical_fit || arg_ellipsoidal_fit)
    {
        printf("Using Field Strength: %.5f\n\n", arg_field_strength.value());
    }

    if (arg_spatial_coverage)
    {
        printf("Spatial Coverage: %.5f%%\n\n", microstrain_mag_cal::calculateSpatialCoverage(points, initial_offset));
    }

    if (arg_spherical_fit)
    {
        const microstrain_mag_cal::FitResult fit_result =
            microstrain_mag_cal::fitSphere(points, arg_field_strength.value(), initial_offset);

        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, arg_field_strength.value());

        displayFitResult("Spherical Fit", fit_result, fit_RMSE);
    }

    if (arg_ellipsoidal_fit)
    {
        const microstrain_mag_cal::FitResult fit_result =
            microstrain_mag_cal::fitEllipsoid(points, arg_field_strength.value(), initial_offset);

        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, arg_field_strength.value());

        displayFitResult("Ellipsoidal Fit", fit_result, fit_RMSE);
    }

    return 0;
}
