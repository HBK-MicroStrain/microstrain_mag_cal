#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <mip/mip_interface.hpp>
#include "mip/definitions/commands_3dm.hpp"
//#include "mip/definitions/commands_base.hpp"

static constexpr const char *PORT_NAME = "COM37";
static constexpr uint32_t    BAUDRATE  = 115200;

float HARD_IRON_OFFSET[3] {
    -0.0851329,
     0.0874525,
     0.2101710
};
float SOFT_IRON_MATRIX[9] {
     0.84556900, -0.0505522, -0.00461753,
    -0.05055220,  0.8868560,  0.05606970,
    -0.00461753,  0.0560697,  0.98104600
};

std::filesystem::path json_file = "C:/Users/AFARRELL/Downloads/ellipsoidal_fit.json";


int main()
{
    printf("Reading from file: %s...\n", json_file.string().c_str());

    std::ifstream file(json_file);
    if (!file.is_open())
    {
        printf("ERROR: Could not open file: %s\n", json_file.string().c_str());
        return 1;
    }

    nlohmann::json parsed_json = nlohmann::json::parse(file);

    printf("Applying calibration...\n");

    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    if (!connection.connect())
    {
        printf("ERROR: Connection Failed.\n");
    }

    mip::Interface device(&connection, mip::C::mip_timeout_from_baudrate(BAUDRATE), 2000);

    if (!mip::commands_3dm::writeMagHardIronOffset(device, HARD_IRON_OFFSET))
    {
        printf("ERROR: Updating hard-iron offset failed.");

        if (!mip::commands_3dm::saveMagHardIronOffset(device))
        {
            printf("ERROR: Saving hard-iron offset failed.");
        }
    }

    if (!mip::commands_3dm::writeMagSoftIronMatrix(device, SOFT_IRON_MATRIX))
    {
        printf("ERROR: Updating soft-iron matrix failed.");

        if (!mip::commands_3dm::saveMagSoftIronMatrix(device))
        {
            printf("ERROR: Saving soft-iron matrix failed.");
        }
    }

    printf("Done.\n");
}
