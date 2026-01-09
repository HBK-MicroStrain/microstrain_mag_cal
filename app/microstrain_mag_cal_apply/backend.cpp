#include "backend.hpp"

#include <mip/definitions/commands_3dm.hpp>

namespace backend
{
    /// @brief Connects to a device on the given port, providing an interface to work with it.
    std::optional<DeviceConnection> connectToDevice(const std::string& port_name, const std::uint32_t baudrate)
    {
        auto connection = std::make_unique<microstrain::connections::SerialConnection>(port_name, baudrate);

        if (!connection->connect())
        {
            return std::nullopt;
        }

        return std::optional<DeviceConnection>(std::in_place, std::move(connection), baudrate);
    }

    /// @brief Reads the current calibration from the device and converts it to work with the app.
    std::optional<microstrain_mag_cal::FitResult> readCalibrationFromDevice(mip::Interface& device_interface)
    {
        float hard_iron_from_device[3];
        float soft_iron_from_device[9];

        if (!mip::commands_3dm::readMagSoftIronMatrix(device_interface, soft_iron_from_device) ||
            !mip::commands_3dm::readMagHardIronOffset(device_interface, hard_iron_from_device))
        {
            return std::nullopt;
        }

        microstrain_mag_cal::FitResult fit_result;
        fit_result.soft_iron_matrix = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(soft_iron_from_device).cast<double>();
        fit_result.hard_iron_offset = Eigen::Map<const Eigen::Vector3f>(hard_iron_from_device).cast<double>();

        return std::optional(std::move(fit_result));
    }
}
