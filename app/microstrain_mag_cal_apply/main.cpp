#include <filesystem>
#include <iostream>

#include <nlohmann/json.hpp>

#include <microstrain/connections/serial/serial_connection.hpp>
#include <microstrain_mag_cal/analysis.hpp>
#include <mip/mip_interface.hpp>
#include "mip/definitions/commands_3dm.hpp"

static constexpr const char *PORT_NAME = "COM37";
static constexpr uint32_t    BAUDRATE  = 115200;

std::filesystem::path json_file = "C:/Users/AFARRELL/Downloads/ellipsoidal_fit.json";


// TODO: Where to put this? Can move to app backend or is there some microstrain utility module or something
//       where this could be useful?
template<typename T, typename Derived>
std::vector<T> toArray(const Eigen::MatrixBase<Derived>& matrix)
{
    using MatrixTemplate = Eigen::Matrix<T, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>;
    MatrixTemplate converted = matrix.template cast<T>().eval();

    return std::vector<T>(converted.data(), converted.data() + converted.size());
}


int main()
{
    microstrain_mag_cal::FitResult fit_result = microstrain_mag_cal::deserializeFitResultFromFile(json_file);

    if (fit_result.error == microstrain_mag_cal::FitResult::Error::DESERIALIZATION_COULD_NOT_OPEN_FILE)
    {
        printf("ERROR: Could not open file to deserialize: %s", json_file.filename().string().c_str());

        return 1;
    }

    if (fit_result.error != microstrain_mag_cal::FitResult::Error::NONE)
    {
        std::cout << "WARNING: Applying calibration that contains error code: " << magic_enum::enum_name(fit_result.error);
    }

    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    if (!connection.connect())
    {
        printf("ERROR: Connection Failed.\n");
    }

    mip::Interface device(&connection, mip::C::mip_timeout_from_baudrate(BAUDRATE), 2000);

    if (!mip::commands_3dm::writeMagHardIronOffset(device, toArray<float>(fit_result.hard_iron_offset).data()))
    {
        printf("ERROR: Updating hard-iron offset failed.");

        if (!mip::commands_3dm::saveMagHardIronOffset(device))
        {
            printf("ERROR: Saving hard-iron offset failed.");
        }
    }

    if (!mip::commands_3dm::writeMagSoftIronMatrix(device, toArray<float>(fit_result.soft_iron_matrix).data()))
    {
        printf("ERROR: Updating soft-iron matrix failed.");

        if (!mip::commands_3dm::saveMagSoftIronMatrix(device))
        {
            printf("ERROR: Saving soft-iron matrix failed.");
        }
    }

    return 0;
}
