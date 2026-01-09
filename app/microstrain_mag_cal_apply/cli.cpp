#include "cli.hpp"

#include <iostream>


namespace cli
{
    ProgramArgs::ProgramArgs(char** argv)
    {
        app.description("Tool for applying a new magnetometer calibration to a device.\n\n"
                        "The new calibration is composed with the existing calibration by default.");
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
        app.add_flag("-o,--overwrite-calibration", overwrite, "Overwrite old calibration instead of composing.")
            ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
    }

    void displayFailedConnectionInformation(const std::string_view port_name, const std::uint32_t baudrate)
    {
        std::cerr << "ERROR: Failed to connect to device with\n"
                  << "    --->     Port: " << port_name << '\n'
                  << "    ---> Baudrate: " << baudrate << '\n';
    }
}
