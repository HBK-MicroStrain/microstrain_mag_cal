#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <microstrain_mag_cal/calibration.hpp>
#include <mip/mip_interface.hpp>
#include "mip/definitions/commands_3dm.hpp"


// TODO: Move to view module
struct ProgramArgs
{
    std::filesystem::path calibration_filepath;

    std::string port_name;
    std::uint32_t baudrate = 115200;
};

// TODO: Move to view module
void setup_argument_parser(CLI::App& app, ProgramArgs& args, char* argv[])
{
    app.description("Tool for applying a magnetometer calibration to a device.");
    app.usage("Usage: " + std::filesystem::path(argv[0]).filename().string() + " <calibration_file> <port_name> [OPTIONS]");

    app.add_option("calibration_file", args.calibration_filepath, "JSON file containing a calibration to apply.")
        ->check(CLI::ExistingFile)
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();
    app.add_option("port_name", args.port_name, "Name of the port for the device to connect to.")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw)
        ->required();

    app.add_option("-b,--baudrate", args.baudrate, "Baudrate of the device to connect to (defaults to 115200).")
        ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
}


int main(const int argc, char **argv)
{
    ProgramArgs args;
    CLI::App app;
    setup_argument_parser(app, args, argv);
    CLI11_PARSE(app, argc, argv);

    microstrain_mag_cal::FitResult fit_result = microstrain_mag_cal::deserializeFitResultFromFile(args.calibration_filepath);

    if (fit_result.error == microstrain_mag_cal::FitResult::Error::DESERIALIZATION_COULD_NOT_OPEN_FILE)
    {
        printf("ERROR: Could not open file to deserialize: %s\n", args.calibration_filepath.filename().string().c_str());

        return 1;
    }

    if (fit_result.error != microstrain_mag_cal::FitResult::Error::NONE)
    {
        std::cout << "ERROR: Calibration contains error: " << magic_enum::enum_name(fit_result.error) << "\n";

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

    const std::vector<float> hard_iron_offset = microstrain_mag_cal::toVector<float>(fit_result.hard_iron_offset);
    const std::vector<float> soft_iron_matrix = microstrain_mag_cal::toVector<float>(fit_result.soft_iron_matrix);

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

    return 0;
}
