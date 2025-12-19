#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <magic_enum/magic_enum.hpp>

#include <backend.hpp>
#include <cli.hpp>
#include <microstrain_mag_cal/analysis.hpp>
#include <microstrain_mag_cal/calibration.hpp>


using microstrain_mag_cal::FitResult;


struct ProgramArgs
{
    explicit ProgramArgs(char** argv)
    {
        app.description("Tool to fit magnetometer calibrations for a device.");
        app.usage("Usage: " + std::filesystem::path(argv[0]).filename().string() + " <file> [OPTIONS]");

        app.add_option("file", input_data_filepath, "A binary file containing mip data to read from.")
            ->check(CLI::ExistingFile)
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
            ->required();

        CLI::Option_group* core = app.add_option_group("Core", "Core functionality of the tool.");

        core->add_flag("-a,--display-analysis", display_analysis, "Display comprehensive analysis output.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
        core->add_flag("-s,--spherical-fit", spherical_fit, "Perform a spherical fit on the input data.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
        core->add_flag("-e,--ellipsoidal-fit", ellipsoidal_fit, "Perform an ellipsoidal fit on the input data.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

        core->require_option(1, 0);

        app.add_option("-f,--field-strength", field_strength, "Field strength to use as a reference instead of using the measured field strength.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

        app.add_option("-j,--output-json", output_json_directory, "Write the resulting calibration(s) to JSON file(s) in the given directory.")
            ->check(CLI::ExistingDirectory)
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    }

    CLI::App app{};

    std::filesystem::path input_data_filepath;
    bool display_analysis = false;
    bool spherical_fit = false;
    bool ellipsoidal_fit = false;
    std::optional<double> field_strength;
    std::filesystem::path output_json_directory;
};


int main(const int argc, char **argv)
{
    ProgramArgs args(argv);
    CLI11_PARSE(args.app, argc, argv);

    const std::optional<backend::MappedBinaryData> mapped_data = backend::mapBinaryFile(args.input_data_filepath);
    assert(mapped_data.has_value());

    const microstrain_mag_cal::PointManager point_manager = backend::extractPointsFromRawData(mapped_data->view, args.field_strength);
    const Eigen::MatrixX3d points = point_manager.getMatrix();
    const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

    if (!args.field_strength.has_value())
    {
        printf("\nNOTE: Using estimate for field strength.\n");

        args.field_strength = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(points, initial_offset);
    }

    if (args.display_analysis)
    {
        printf("\n");
        printf("--------------------------------------------------\n");
        printf("Analysis:\n");
        printf("--------------------------------------------------\n");
        printf("  Used Points       : %s\n", cli::getPointUsageDisplay(point_manager).c_str());
        printf("  Spatial Coverage  : %.5f%%\n", microstrain_mag_cal::calculateSpatialCoverage(points, initial_offset));
        printf("  Field Strength    : %.5f\n", args.field_strength.value());
        printf("  Initial Offset    : [%.5f, %.5f, %.5f]\n", initial_offset.x(), initial_offset.y(), initial_offset.z());
    }

    if (args.spherical_fit)
    {
        const FitResult fit_result = microstrain_mag_cal::fitSphere(points, args.field_strength.value(), initial_offset);
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        cli::displayFitResult("Spherical Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "spherical_fit.json", fit_result);
        }
    }

    if (args.ellipsoidal_fit)
    {
        const FitResult fit_result = microstrain_mag_cal::fitEllipsoid(points, args.field_strength.value(), initial_offset);
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        cli::displayFitResult("Ellipsoidal Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "ellipsoidal_fit.json", fit_result);
        }
    }

    return 0;
}
