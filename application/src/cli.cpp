#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <mio/mmap.hpp>

#include <mag_cal_core.hpp>

int main(const int argc, char **argv)
{
    std::filesystem::path filepath;
    bool spherical_fit = false;
    bool ellipsoidal_fit = false;

    CLI::App app{"MVP converting the mag cal logic from InertialConnect into a standalone application."};

    app.add_option("-f,--file", filepath, "A binary file containing mip data to read from.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_flag("-s,--spherical-fit", spherical_fit, "Calculate the spherical fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    app.add_flag("-e,--ellipsoidal-fit", ellipsoidal_fit, "Calculate the ellipsoidal fit of the input data.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    CLI11_PARSE(app, argc, argv);

    // Map a read-only view of the given file.
    std::error_code error;
    const mio::mmap_source file_mapping = mio::make_mmap_source("test_file_for_offline_mag.bin", error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_mapping.data());
    const microstrain::ConstU8ArrayView data_view(data, file_mapping.size());

    Eigen::MatrixX3d point_matrix = mag_cal_core::extractPointMatrixFromRawData(data_view);

    // TODO: Use point_matrix as input to calibration functions

    return 0;
}
