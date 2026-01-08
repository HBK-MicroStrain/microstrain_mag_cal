#pragma once

#include <CLI/CLI.hpp>


namespace cli
{
    struct ProgramArgs
    {
        explicit ProgramArgs(char** argv);

        CLI::App app{};

        std::filesystem::path calibration_filepath;
        std::string port_name;
        std::uint32_t baudrate = 115200;
    };
}
