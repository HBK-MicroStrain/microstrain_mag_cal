#include "backend.hpp"


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
}
