#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_mag_cal_apply/composed_coefficients.hpp>
#include <mip/mip_interface.hpp>
#include "mip/definitions/commands_3dm.hpp"


// TODO: Move to cli module
struct ProgramArgs
{
    explicit ProgramArgs(char** argv)
    {
        app.description("Tool for applying a magnetometer calibration to a device.");
        app.usage("Usage: " + std::filesystem::path(argv[0]).filename().string() + " <calibration_file> <port_name> [OPTIONS]");

        app.add_option("calibration_file", calibration_filepath, "JSON file containing a calibration to apply.")
            ->check(CLI::ExistingFile)
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
            ->required();
        app.add_option("port_name", port_name, "Name of the port for the device to connect to.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
            ->required();

        app.add_option("-b,--baudrate", baudrate, "Baudrate of the device to connect to (defaults to 115200).")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    }

    CLI::App app{};

    std::filesystem::path calibration_filepath;
    std::string port_name;
    std::uint32_t baudrate = 115200;
};


int main(const int argc, char **argv)
{
    ProgramArgs args(argv);
    CLI11_PARSE(args.app, argc, argv);

    const microstrain_mag_cal::FitResult new_fit = microstrain_mag_cal::deserializeFitResultFromFile(args.calibration_filepath);

    if (new_fit.error == microstrain_mag_cal::FitResult::Error::DESERIALIZATION_COULD_NOT_OPEN_FILE)
    {
        printf("ERROR: Could not open file to deserialize: %s\n", args.calibration_filepath.filename().string().c_str());

        return 1;
    }

    if (new_fit.error != microstrain_mag_cal::FitResult::Error::NONE)
    {
        std::cout << "ERROR: Calibration contains error: " << magic_enum::enum_name(new_fit.error) << "\n";

        return 1;
    }

    microstrain::connections::SerialConnection connection(args.port_name, args.baudrate);

    if (!connection.connect())
    {
        printf("ERROR: Failed to connect to device with\n");
        printf("    --->     Port: %s\n", args.port_name.c_str());
        printf("    ---> Baudrate: %d\n", args.baudrate);

        return 1;
    }

    mip::Interface device(&connection, mip::C::mip_timeout_from_baudrate(args.baudrate), 2000);

    microstrain_mag_cal::FitResult old_fit;
    float hard_iron_from_device[3];
    float soft_iron_from_device[9];

    if (!mip::commands_3dm::readMagHardIronOffset(device, hard_iron_from_device))
    {
        printf("ERROR: Reading old hard-iron offset from device failed.\n");

        return 1;
    }

    if (!mip::commands_3dm::readMagSoftIronMatrix(device, soft_iron_from_device))
    {
        printf("ERROR: Reading old soft-iron-matrix from device failed.\n");

        return 1;
    }

    old_fit.soft_iron_matrix = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(soft_iron_from_device).cast<double>();
    old_fit.hard_iron_offset = Eigen::Map<const Eigen::Vector3f>(hard_iron_from_device).cast<double>();
    const microstrain_mag_cal::FitResult composed_fit = microstrain_mag_cal::composeCorrections(old_fit, new_fit);

    const std::vector<float> soft_iron_matrix = microstrain_mag_cal::toVector<float>(composed_fit.soft_iron_matrix);
    const std::vector<float> hard_iron_offset = microstrain_mag_cal::toVector<float>(composed_fit.hard_iron_offset);

    if (!mip::commands_3dm::writeMagSoftIronMatrix(device, soft_iron_matrix.data()))
    {
        printf("ERROR: Writing soft-iron matrix failed.\n");

        return 1;
    }

    if (!mip::commands_3dm::saveMagSoftIronMatrix(device))
    {
        printf("ERROR: Writing soft-iron matrix failed.\n");

        return 1;
    }

    if (!mip::commands_3dm::writeMagHardIronOffset(device, hard_iron_offset.data()))
    {
        printf("ERROR: Writing hard-iron offset failed.\n");

        return 1;
    }

    if (!mip::commands_3dm::saveMagHardIronOffset(device))
    {
        printf("ERROR: Saving hard-iron offset failed.\n");

        return 1;
    }

    return 0;
}
