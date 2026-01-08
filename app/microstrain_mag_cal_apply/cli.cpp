#include "cli.hpp"

#include <iostream>

namespace cli
{
    ProgramArgs::ProgramArgs(char** argv)
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
}
