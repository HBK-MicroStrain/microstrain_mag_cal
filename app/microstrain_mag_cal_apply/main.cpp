#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_mag_cal_apply/composed_coefficients.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <nlohmann/json.hpp>

#include "backend.hpp"
#include "cli.hpp"


int main(const int argc, char **argv)
{
    cli::ProgramArgs args(argv);
    CLI11_PARSE(args.app, argc, argv);

    const microstrain_mag_cal::FitResult new_fit = microstrain_mag_cal::deserializeFitResultFromFile(args.calibration_filepath);

    if (new_fit.error != microstrain_mag_cal::FitResult::Error::NONE)
    {
        std::cerr << "ERROR: Calibration contains error ---> " << magic_enum::enum_name(new_fit.error) << "\n";

        return 1;
    }

    std::optional<backend::DeviceConnection> device_connection = backend::connectToDevice(args.port_name, args.baudrate);

    if (!device_connection)
    {
        cli::displayFailedConnectionInformation(args.port_name, args.baudrate);

        return 1;
    }

    const std::optional<microstrain_mag_cal::FitResult> old_fit = backend::readCalibrationFromDevice(device_connection->interface);

    if (!old_fit)
    {
        std::cerr << "ERROR: Reading calibration from the device failed.\n";

        return 1;
    }

    const microstrain_mag_cal::FitResult composed_fit = microstrain_mag_cal::composeCorrections(old_fit.value(), new_fit);

    if (!backend::writeCalibrationToDevice(device_connection->interface, composed_fit))
    {
        std::cerr << "ERROR: Writing calibration to the device failed.\n";

        return 1;
    }

    printf("Success.\n");

    return 0;
}
