#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <mio/mmap.hpp>

#include <mag_cal_core.hpp>


class UsageFormatter final : public CLI::Formatter
{
public:
    std::string make_usage(const CLI::App* app, const std::string name) const override
    {
       return "USAGE: " + name + (app->get_help_ptr() != nullptr ? " [OPTIONS]" : "") + "\n";
    }
};


int main(const int argc, char **argv)
{
    /*** Commandline arguments ***/

    std::filesystem::path filepath;
    bool spherical_fit = false;
    bool ellipsoidal_fit = false;

    CLI::App app{"MVP converting the mag cal logic from InertialConnect into a standalone application."};
    app.formatter(std::make_shared<UsageFormatter>());

    app.add_option("-f,--file", filepath, "A binary file containing mip data to read from.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_flag("-s,--spherical-fit", spherical_fit, "Calculate the spherical fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-e,--ellipsoidal-fit", ellipsoidal_fit, "Calculate the ellipsoidal fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    CLI11_PARSE(app, argc, argv);

    /*** Read-only view of the input file ***/

    std::error_code error;
    const mio::mmap_source file_view = mio::make_mmap_source(filepath.string(), error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_view.data());
    const microstrain::ConstU8ArrayView data_view(data, file_view.size());

    /*** Point matrix extraction ***/

    Eigen::MatrixX3d point_matrix = mag_cal_core::extractPointMatrixFromRawData(data_view);

    /*** Fit algorithms ***/

    if (spherical_fit)
    {
        // TODO: Run fit algorithm
    }

    if (ellipsoidal_fit)
    {
        // TODO: Run fit algorithm
    }

    return 0;
}
