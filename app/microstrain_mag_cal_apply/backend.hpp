#pragma once

#include <microstrain/connections/serial/serial_connection.hpp>
#include <mip/mip_interface.hpp>

#include <microstrain_mag_cal/calibration.hpp>


namespace backend
{
    struct DeviceConnection
    {
        DeviceConnection(microstrain::connections::SerialConnection connection, std::uint32_t baudrate)
        : connection(std::move(connection)),
          interface(&connection, mip::C::mip_timeout_from_baudrate(baudrate), 2000) {}

        microstrain::connections::SerialConnection connection;
        mip::Interface interface;
    };

    std::optional<DeviceConnection> connectToDevice(const std::string& port_name, std::uint32_t baudrate);
}
