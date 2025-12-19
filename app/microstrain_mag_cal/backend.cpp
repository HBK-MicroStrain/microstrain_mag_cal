#include <backend.hpp>

#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_mag_cal/preprocessing.hpp>
#include <mip/mip_parser.hpp>
#include <mip/definitions/data_sensor.hpp>


using ScaledMag = mip::data_sensor::ScaledMag;


namespace backend
{
    /// @brief Returns a read-only mapping of the file and a view of the bytes.
    std::optional<MappedBinaryData> mapBinaryFile(const std::filesystem::path& filepath)
    {
        std::error_code error;
        mio::mmap_source mapping = mio::make_mmap_source(filepath.string(), error);

        if (error)
        {
            return std::nullopt;
        }

        const microstrain::ConstU8ArrayView view(reinterpret_cast<const uint8_t*>(mapping.data()), mapping.size());

        return MappedBinaryData{std::move(mapping), view};
    }

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
                std::array<float, 3> temp{};

                for (uint8_t i = 0; i < 3; ++i)
                {
                    if (const bool success = serializer.extract(temp[i]); !success)
                    {
                        assert(false);
                        throw std::runtime_error("ERROR: Serialized data parsing failed.");
                    }
                }

                m_point_manager->addPoint(temp);
            }
        }

        return true;
    }

    /// Extracts a matrix of points from raw binary data.
    microstrain_mag_cal::PointManager extractPointsFromRawData(
        const microstrain::ConstU8ArrayView &data_view,
        const std::optional<double> reference_field_strength)
    {
        // More conservative default if we don't know the field strength
        double voxel_size = microstrain_mag_cal::VoxelGrid::DEFAULT_VOXEL_SIZE;

        // More accurate adaptive filtering if field strength available.
        // This should give faster and more accurate fits, even if the spatial coverage is lower.
        if (reference_field_strength.has_value())
        {
            voxel_size *= reference_field_strength.value() * 0.03;
        }

        const microstrain_mag_cal::VoxelGrid unique_point_grid(voxel_size);
        microstrain_mag_cal::PointManager point_manager(unique_point_grid);

        // We aren't working with a device, so the timeouts and timestamp aren't needed.
        mip::Parser parser(&extractPoints, &point_manager, 0);
        parser.parse(data_view.data(), data_view.size(), 0);

        return point_manager;
    }
}
