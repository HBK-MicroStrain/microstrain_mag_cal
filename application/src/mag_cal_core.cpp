#include <mag_cal_core.hpp>

#include <microstrain_mag_cal.hpp>
#include <mip/mip_parser.hpp>
#include <mip/definitions/data_sensor.hpp>

using ScaledMag = mip::data_sensor::ScaledMag;


// Extracts the points into the point manager. Called by the mip parser for each packet found.
bool extractPoints(void *point_manager, const mip::PacketView *packet_view, const mip::Timestamp /* timestamp */)
{
    assert(packet_view);

    microstrain_mag_cal::PointManager *m_point_manager = static_cast<microstrain_mag_cal::PointManager*>(point_manager);

    if (packet_view->descriptorSet() == ScaledMag::DESCRIPTOR_SET)
    {
        for (const mip::FieldView &field : *packet_view)
        {
            if (field.fieldDescriptor() != ScaledMag::FIELD_DESCRIPTOR)
            {
                continue;
            }

            mip::Serializer serializer(field.payload());
            std::array<float, 3> temp;

            for (uint8_t i = 0; i < 3; ++i)
            {
                assert(serializer.extract(temp[i]));
            }

            m_point_manager->addPoint(temp);
        }
    }

    return true;
}

namespace mag_cal_core
{
    Eigen::MatrixX3d extractPointMatrixFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        const std::optional<double> reference_field_strength)
    {
        double voxel_size = 0.5;  // Moderate filtering default

        // Much better filtering, if available
        if (reference_field_strength.has_value())
        {
            voxel_size = reference_field_strength.value() * 0.05;
        }

        const microstrain_mag_cal::VoxelGrid unique_point_grid(voxel_size);
        microstrain_mag_cal::PointManager point_manager(unique_point_grid);

        // We aren't working with a device, so the timeouts and timestamp aren't needed.
        mip::Parser parser(&extractPoints, &point_manager, 0);
        parser.parse(data_view.data(), data_view.size(), 0);

        return point_manager.getMatrix();
    }
}
