#include <mag_cal_core.hpp>

#include <mip/mip_parser.hpp>
#include <mip/definitions/data_sensor.hpp>

using ScaledMag = mip::data_sensor::ScaledMag;


// Called by the mip parser for each packet found.
// Flattened list ---> (x1, y1, z1, ..., xN, yN, zN).
bool extractPointsIntoFlattenedList(void *flattened_points_out, const mip::PacketView *packet_view, const mip::Timestamp timestamp)
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
            if (field.fieldDescriptor() != ScaledMag::FIELD_DESCRIPTOR)
            {
                continue;
            }

            mip::Serializer serializer(field.payload());
            float temp;

            for (uint8_t i = 0; i < 3; ++i)
            {
                assert(serializer.extract(temp));
                flattened_points->push_back(temp);
            }
        }
    }

    return true;
}

namespace mag_cal_core
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, const size_t data_size)
    {
        // Extract point vectors as flattened list of points (x1, y1, z1, ..., xN, yN, zN)
        // We aren't working with a device, so the timeouts and timestamp aren't needed.
        std::vector<double> flattened_points;
        mip::Parser parser(&extractPointsIntoFlattenedList, &flattened_points, 0);
        parser.parse(data, data_size, 0);

        // Zero-copy map to matrix
        return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
            flattened_points.data(),
            static_cast<Eigen::Index>(flattened_points.size()) / 3,
            3
        );
    }
}
