#include <iostream>

#include "mio/mmap.hpp"
#include "mip/mip_interface.hpp"
#include "mip/definitions/data_sensor.hpp"

using ScaledMag = mip::data_sensor::ScaledMag;


// TODO: Implement
void addPointToMatrix(void *matrix, const mip::FieldView &field_view, mip::Timestamp timestamp)
{

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

    // TODO: Set up two-pass approach to pre-allocate matrix
    // Extract a matrix of points from the raw binary data.
    // We aren't working with a device, so the timeouts and timestamp aren't needed.
    mip::Interface data_parser(0, 0);
    mip::DispatchHandler fieldHandler;
    data_parser.registerFieldCallback<&addPointToMatrix>(fieldHandler, ScaledMag::DESCRIPTOR_SET, ScaledMag::FIELD_DESCRIPTOR, nullptr);
    data_parser.inputBytes(data, data_size, 0);

    return 0;
}