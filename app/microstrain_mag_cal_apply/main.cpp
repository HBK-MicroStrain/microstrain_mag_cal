#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <microstrain_mag_cal/analysis.hpp>
#include <mip/mip_interface.hpp>
#include "mip/definitions/commands_3dm.hpp"

// TODO: Where to put this? Can move to app backend or is there some microstrain utility module or something
//       where this could be useful?
// TODO: Add test with same data types as soft-iron matrix and hard-iron offset to check row-major/column-major
//       compatibility (i.e. want to make sure it's in the proper format when written to the device.
template<typename T, typename Derived>
std::vector<T> toArray(const Eigen::MatrixBase<Derived>& matrix)
{
    using MatrixTemplate = Eigen::Matrix<T, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>;
    MatrixTemplate converted = matrix.template cast<T>().eval();

    return std::vector<T>(converted.data(), converted.data() + converted.size());
}

int main(const int argc, char **argv)
{
    // Required
    std::filesystem::path arg_calibration_filepath;
    std::string arg_port_name;

    // Optional
    std::uint32_t arg_baudrate = 115200;

    CLI::App app{"Command-line tool to apply a magnetometer calibration to a device."};
    app.usage("Usage: " + std::filesystem::path(argv[0]).filename().string() + " <calibration_file> <port_name> [OPTIONS]");

    app.add_option("calibration_file", arg_calibration_filepath, "JSON file containing a calibration to apply.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_option("port_name", arg_port_name, "Name of the port for the device to connect to.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_option("-b,--baudrate", arg_baudrate, "Baudrate of the device to connect to (defaults to 115200).")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);

    CLI11_PARSE(app, argc, argv);

    microstrain_mag_cal::FitResult fit_result = microstrain_mag_cal::deserializeFitResultFromFile(arg_calibration_filepath);

    if (fit_result.error == microstrain_mag_cal::FitResult::Error::DESERIALIZATION_COULD_NOT_OPEN_FILE)
    {
        printf("ERROR: Could not open file to deserialize: %s", arg_calibration_filepath.filename().string().c_str());

        return 1;
    }

    if (fit_result.error != microstrain_mag_cal::FitResult::Error::NONE)
    {
        std::cout << "ERROR: Calibration contains error: " << magic_enum::enum_name(fit_result.error) << "\n";

        return 1;
    }

    microstrain::connections::SerialConnection connection(arg_port_name, arg_baudrate);

    if (!connection.connect())
    {
        printf("ERROR: Failed to connect to device with\n");
        printf("    --->     Port: %s\n", arg_port_name.c_str());
        printf("    ---> Baudrate: %d", arg_baudrate);

        return 1;
    }

    mip::Interface device(&connection, mip::C::mip_timeout_from_baudrate(arg_baudrate), 2000);

    if (!mip::commands_3dm::writeMagHardIronOffset(device, toArray<float>(fit_result.hard_iron_offset).data()))
    {
        printf("ERROR: Writing hard-iron offset failed.");

        return 1;
    }

    if (!mip::commands_3dm::saveMagHardIronOffset(device))
    {
        printf("ERROR: Saving hard-iron offset failed.");

        return 1;
    }

    if (!mip::commands_3dm::writeMagSoftIronMatrix(device, toArray<float>(fit_result.soft_iron_matrix).data()))
    {
        printf("ERROR: Writing soft-iron matrix failed.");

        return 1;
    }

    if (!mip::commands_3dm::saveMagSoftIronMatrix(device))
    {
        printf("ERROR: Writing soft-iron matrix failed.");

        return 1;
    }

    return 0;
}
