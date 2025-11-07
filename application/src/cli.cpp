#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <mio/mmap.hpp>

#include <mag_cal_core.hpp>

#include "microstrain_mag_cal.hpp"


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

// Console output after the fitting algorithms are run
void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result)
{
    static constexpr int MIN_SUPPORTED_TERMINAL_WIDTH = 50;

    printf("%s\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());
    printf("%s\n", fit_name.data());
    printf("%s\n\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());

    printf("Fit Result: %s\n\n", result.succeeded ? "SUCCEEDED" : "FAILED");

    printf("Soft-Iron Matrix:\n");
    std::cout << result.soft_iron_matrix << "\n\n";

    printf("Hard-Iron Offset:\n");
    std::cout << result.hard_iron_offset << "\n\n";
}


int main(const int argc, char **argv)
{
    /*** Parse commandline arguments ***/

    std::filesystem::path filepath;
    bool spherical_fit = false;
    bool ellipsoidal_fit = false;

    CLI::App app{"MVP converting the mag cal logic from InertialConnect into a standalone application."};
    app.formatter(std::make_shared<HelpMessageFormatter>());

    app.add_option("-f,--file", filepath, "A binary file containing mip data to read from.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_flag("-s,--spherical-fit", spherical_fit, "Calculate the spherical fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-e,--ellipsoidal-fit", ellipsoidal_fit, "Calculate the ellipsoidal fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    CLI11_PARSE(app, argc, argv);

    /*** Create a read-only view of the input file ***/

    std::error_code error;
    const mio::mmap_source file_view = mio::make_mmap_source(filepath.string(), error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_view.data());
    const microstrain::ConstU8ArrayView data_view(data, file_view.size());

    /*** Run the calculations ***/

    const Eigen::MatrixX3d points = mag_cal_core::extractPointMatrixFromRawData(data_view);
    const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

    // TODO: Set to check for argument first
    const double field_strength = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(points, initial_offset);

    if (spherical_fit)
    {
        const microstrain_mag_cal::FitResult result =
            microstrain_mag_cal::fitSphere(points, field_strength, initial_offset);

        displayFitResult("Spherical Fit", result);
    }

    if (ellipsoidal_fit)
    {
        const microstrain_mag_cal::FitResult result =
            microstrain_mag_cal::fitEllipsoid(points, field_strength, initial_offset);

        displayFitResult("Ellipsoidal Fit", result);
    }

    return 0;
}
