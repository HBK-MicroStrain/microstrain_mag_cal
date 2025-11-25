#include <iostream>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <mip/mip_interface.hpp>

#include "mip/definitions/commands_base.hpp"

static constexpr const char *PORT_NAME = "COM37";
static constexpr uint32_t BAUDRATE = 115200;


int main()
{
    std::cout << "Applying calibration...\n";

    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    if (!connection.connect())
    {
        std::cout << "Connection Failed!!!\n";
    }

    mip::Interface device(&connection, mip::C::mip_timeout_from_baudrate(BAUDRATE), 2000);

    if (!mip::commands_base::ping(device))
    {
        std::cout << "Ping Failed!!!\n";
    }

    std::cout << "Done...\n";
}
