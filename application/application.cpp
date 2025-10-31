#include <iostream>

#include "mio/mmap.hpp"
#include "mip/mip_interface.hpp"
#include "mip/definitions/data_sensor.hpp"

using ScaledMag = mip::data_sensor::ScaledMag;


/// Creates a read-only memory-mapped view of the given file.
mio::mmap_source map_file(const std::string &file_path, std::error_code &error)
{
    mio::mmap_source file_mapping = mio::make_mmap_source(file_path, error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
    }

    return file_mapping;
}

// TODO: Implement
void addPointToMatrix(void *matrix, const mip::FieldView &field_view, mip::Timestamp timestamp)
{

}


int main()
{
    std::error_code error;
    const mio::mmap_source file_mapping = map_file("test_file_for_offline_mag.bin", error);

    if (error)
    {
        return 1;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(file_mapping.data());
    const size_t data_size = file_mapping.size();

    // Extract a matrix of points from the raw binary data.
    // We aren't working with a device, so the timeouts and timestamp aren't needed.
    mip::Interface data_parser(0, 0);
    mip::DispatchHandler fieldHandler;
    data_parser.registerFieldCallback<&addPointToMatrix>(fieldHandler, ScaledMag::DESCRIPTOR_SET, ScaledMag::FIELD_DESCRIPTOR, nullptr);
    data_parser.inputBytes(data, data_size, 0);

    return 0;
}