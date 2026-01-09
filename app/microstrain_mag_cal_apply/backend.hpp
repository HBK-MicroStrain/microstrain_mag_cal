#pragma once

#include <microstrain/connections/serial/serial_connection.hpp>
#include <microstrain_mag_cal/calibration.hpp>
#include <mip/mip_interface.hpp>


namespace backend
{
    struct DeviceConnection
    {
        DeviceConnection(std::unique_ptr<microstrain::connections::SerialConnection> connection, std::uint32_t baudrate)
        : connection(std::move(connection)),
          interface(this->connection.get(), mip::C::mip_timeout_from_baudrate(baudrate), 2000) {}

        DeviceConnection(DeviceConnection&&) = delete;
        DeviceConnection& operator=(DeviceConnection&&) = delete;

        std::unique_ptr<microstrain::connections::SerialConnection> connection;
        mip::Interface interface;
    };

    std::optional<DeviceConnection> connectToDevice(const std::string& port_name, std::uint32_t baudrate);

    std::optional<microstrain_mag_cal::FitResult> readCalibrationFromDevice(mip::Interface& device_interface);
    bool writeCalibrationToDevice(mip::Interface& device_interface, const microstrain_mag_cal::FitResult& fit_result);
}
