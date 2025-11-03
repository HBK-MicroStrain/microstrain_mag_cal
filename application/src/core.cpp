#include <core.hpp>

#include <mip/mip_parser.hpp>
#include <mip/definitions/data_sensor.hpp>

using ScaledMag = mip::data_sensor::ScaledMag;


// TODO: Rename
bool extractPoints(void *flattened_points_out, const mip::PacketView *packet_view, const mip::Timestamp timestamp)
{
    (void)timestamp;

    if (packet_view == nullptr)
    {
        return false;
    }

    std::vector<double> *flattened_points = static_cast<std::vector<double>*>(flattened_points_out);

    if (packet_view->descriptorSet() == ScaledMag::DESCRIPTOR_SET)
    {
        for (const mip::FieldView &field : *packet_view)
        {
            if (field.fieldDescriptor() == ScaledMag::FIELD_DESCRIPTOR)
            {
                // TODO: Replace with the improved logic
                float extracted_point[3];
                microstrain::serialization::big_endian::read(field.payload().data(), extracted_point[0]);
                microstrain::serialization::big_endian::read(field.payload().data(), extracted_point[0]);
                microstrain::serialization::big_endian::read(field.payload().data(), extracted_point[0]);

                for (uint8_t i = 0; i < 3; ++i)
                {
                    flattened_points->push_back(static_cast<double>(extracted_point[i]));
                }
            }
        }
    }

    return true;
}

namespace Core
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, const size_t data_size)
    {
        // Extract point vectors as flattened list of points (x1, y1, z1, ..., xN, yN, zN)
        // We aren't working with a device, so the timeouts and timestamp aren't needed.
        std::vector<double> flattened_points;
        mip::Parser parser(&extractPoints, &flattened_points, 0);
        parser.parse(data, data_size, 0);

        // Zero-copy map to matrix
        return Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(
            flattened_points.data(),
            3,
            flattened_points.size()
        );
    }
}
