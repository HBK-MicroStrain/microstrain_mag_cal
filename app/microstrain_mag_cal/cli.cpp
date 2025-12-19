#include <cli.hpp>
#include <iostream>

namespace cli
{
    ProgramArgs::ProgramArgs(char** argv)
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

    // Console output after the fitting algorithms are run
    void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, const double fit_RMSE)
    {
        printf("\n");
        printf("--------------------------------------------------\n");
        printf("%s\n", fit_name.data());
        printf("--------------------------------------------------\n");

        std::string result_output = "SUCCESS";
        if (result.error != microstrain_mag_cal::FitResult::Error::NONE)
        {
            result_output = "FAIL - ";
            const std::string_view error_name = magic_enum::enum_name(result.error);
            result_output += error_name.empty() ? "UNKNOWN" : error_name;
        }
        printf("  Result: %s\n\n", result_output.c_str());

        printf("  Soft-Iron Matrix:\n");
        const Eigen::IOFormat fmt(6, 0, " ", "\n", "  ");
        std::cout << result.soft_iron_matrix.format(fmt) << "\n\n";

        printf("  Hard-Iron Offset:\n");
        std::cout << result.hard_iron_offset.format(fmt) << "\n\n";

        printf("  Fit RMSE: %.5f\n", fit_RMSE);
    }

    // Formats it like this: "NUM USED / NUM TOTAL (RETENTION%)"
    std::string getPointUsageDisplay(const microstrain_mag_cal::PointManager &point_manager)
    {
        const size_t used = point_manager.getNumFilteredPoints();
        const size_t total = point_manager.getNumPointsSeen();
        const double retention = point_manager.getPointRetention();

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%zu / %zu (%.1f%%)", used, total, retention);

        return std::string(buffer);
    }
}
