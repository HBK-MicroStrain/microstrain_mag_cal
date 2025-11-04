#include <iostream>

#include <mio/mmap.hpp>

#include <mag_cal_core.hpp>

int main()
{
    // Map a read-only view of the given file.
    std::error_code error;
    const mio::mmap_source file_mapping = mio::make_mmap_source("test_file_for_offline_mag.bin", error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_mapping.data());
    const microstrain::ConstU8ArrayView data_view(data, file_mapping.size());

    Eigen::MatrixX3d point_matrix = mag_cal_core::extractPointMatrixFromRawData(data_view);

    // TODO: Use point_matrix as input to calibration functions

    return 0;
}
