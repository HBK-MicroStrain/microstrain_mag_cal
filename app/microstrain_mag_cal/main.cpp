#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <magic_enum/magic_enum.hpp>
#include <mio/mmap.hpp>

#include <backend.hpp>
#include <microstrain_mag_cal/analysis.hpp>
#include <microstrain_mag_cal/calibration.hpp>


static constexpr int MIN_SUPPORTED_TERMINAL_WIDTH = 50;


// TODO: Move to view module
// Console output after the fitting algorithms are run
void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, const double fit_RMSE)
{

    printf("%s\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());
    printf("%s\n", fit_name.data());
    printf("%s\n\n", std::string(MIN_SUPPORTED_TERMINAL_WIDTH, '-').data());

    const std::string_view error_name = magic_enum::enum_name(result.error);
    std::cout << "Error: " << (error_name.empty() ? "UNKNOWN" : error_name) << "\n\n";

    printf("Soft-Iron Matrix:\n");
    std::cout << result.soft_iron_matrix << "\n\n";

    printf("Hard-Iron Offset:\n");
    std::cout << result.hard_iron_offset << "\n\n";

    printf("Fit RMSE: %.5f\n\n", fit_RMSE);
}


// TODO: Move to view module
struct ProgramArgs
{
    // Required
    std::filesystem::path input_data_filepath;

    // Optional: Primary Operations
    bool spherical_fit = false;
    bool ellipsoidal_fit = false;

    // Optional: Configuration/Modifiers
    std::optional<double> field_strength;

    // Optional: Output Options
    bool display_analysis = false;
    std::filesystem::path output_json_directory;
};

// TODO: Move to view module
void setup_argument_parser(CLI::App& app, ProgramArgs& args, char* argv[])
{
    app.description("Tool to fit magnetometer calibrations for a device.");
    app.usage("Usage: " + std::filesystem::path(argv[0]).filename().string() + " <file> [OPTIONS]");

    // Required
    app.add_option("file", args.input_data_filepath, "A binary file containing mip data to read from.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();

    // Optional: Primary Operations
    app.add_flag("-s,--spherical-fit", args.spherical_fit, "Perform a spherical fit on the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-e,--ellipsoidal-fit", args.ellipsoidal_fit, "Perform an ellipsoidal fit on the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    // Optional: Configuration/Modifiers
    app.add_option("-r,--reference-field-strength", args.field_strength, "Field strength to use as a reference instead of using the measured field strength.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    // Optional: Output Options
    app.add_flag("-a,--display-analysis", args.display_analysis, "Display comprehensive analysis output.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_option("-j,--output-json", args.output_json_directory, "Write the resulting calibration(s) to JSON file(s) in the given directory.")
        ->check(CLI::ExistingDirectory)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
}

// TODO: Move to backend or microstrain utilities
struct MappedBinaryData
{
    mio::mmap_source mapping;
    microstrain::ConstU8ArrayView view;
};

// TODO: Move to backend or microstrain utilities
// TODO: Move file logic out and add test
std::optional<MappedBinaryData> mapBinaryFile(const std::filesystem::path& filepath)
{
    std::error_code error;
    mio::mmap_source mapping = mio::make_mmap_source(filepath.string(), error);

    if (error)
    {
        return std::nullopt;
    }

    const microstrain::ConstU8ArrayView view(reinterpret_cast<const uint8_t*>(mapping.data()), mapping.size());

    return MappedBinaryData{std::move(mapping), view};
}


int main(const int argc, char **argv)
{
    ProgramArgs args;
    CLI::App app;
    setup_argument_parser(app, args, argv);
    CLI11_PARSE(app, argc, argv);

    const std::optional<MappedBinaryData> mapped_data = mapBinaryFile(args.input_data_filepath);
    assert(mapped_data.has_value());

    const Eigen::MatrixX3d points = backend::extractPointMatrixFromRawData(mapped_data->view, args.field_strength);
    const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

    if (!args.field_strength.has_value())
    {
        // TODO: Output message informing of this
        args.field_strength = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(points, initial_offset);
    }

    if (args.display_analysis)
    {
        printf("--------------------------------------------------\n");
        printf("Analysis:\n");
        printf("--------------------------------------------------\n");
        printf("  Used Points       : %zu\n", static_cast<size_t>(points.rows()));
        printf("  Spatial Coverage  : %.5f%%\n", microstrain_mag_cal::calculateSpatialCoverage(points, initial_offset));
        printf("  Field Strength    : %.5f\n", args.field_strength.value());
    }

    if (args.spherical_fit)
    {
        const microstrain_mag_cal::FitResult fit_result =
            microstrain_mag_cal::fitSphere(points, args.field_strength.value(), initial_offset);

        // TODO: Make reusable function for the rest
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        displayFitResult("Spherical Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "spherical_fit.json", fit_result);
        }
    }

    if (args.ellipsoidal_fit)
    {
        const microstrain_mag_cal::FitResult fit_result =
            microstrain_mag_cal::fitEllipsoid(points, args.field_strength.value(), initial_offset);

        // TODO: Make reusable function for the rest
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        displayFitResult("Ellipsoidal Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "ellipsoidal_fit.json", fit_result);
        }
    }

    return 0;
}
