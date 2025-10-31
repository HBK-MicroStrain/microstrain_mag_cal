#include <iostream>

#include <Eigen/Dense>
#include <mio/mmap.hpp>

#include <microstrain/platform.h>
#include <mip/mip_parser.hpp>
#include <mip/definitions/data_sensor.hpp>


using ScaledMag = mip::data_sensor::ScaledMag;


bool countPointsInRawData(void *num_points, const mip::PacketView *packet_view, mip::Timestamp timestamp)
{
    if (packet_view == nullptr)
    {
        return false;
    }

    if (packet_view->descriptorSet() == ScaledMag::DESCRIPTOR_SET)
    {
        for (const mip::FieldView &field : *packet_view)
        {
            if (field.fieldDescriptor() == ScaledMag::FIELD_DESCRIPTOR)
            {
                size_t *count = static_cast<size_t *>(num_points);
                ++(*count);
            }
        }
    }

    return true;
}


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
    const size_t data_size = file_mapping.size();

    // Pass 1: Pre-allocate matrix
    // We aren't working with a device, so the timeouts and timestamp aren't needed.
    size_t num_points = 0;
    mip::Parser parser(&countPointsInRawData, &num_points, 0);
    parser.parse(data, data_size, 0);
    Eigen::MatrixX3d points(num_points, 3);

    // Pass 2: Extract points from the raw binary data into the matrix.
    // TODO: Implement

    return 0;
}
