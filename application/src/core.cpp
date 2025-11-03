#include <core.hpp>

// TODO: Setup point estimate to pre-allocate data pointer vector
// TODO: Change count callback to extract the data pointers to the vector
/*
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
*/

/*
bool extractPointsToMatrix(Eigen::MatrixX3d &points_matrix, const mip::PacketView &packet_view, mip::Timestamp timestamp)
{
    if (packet_view.descriptorSet() == ScaledMag::DESCRIPTOR_SET)
    {
        for (const mip::FieldView &field_view : packet_view)
        {
            if (field_view.fieldDescriptor() == ScaledMag::FIELD_DESCRIPTOR)
            {
                float extracted_point[3];
                microstrain::serialization::big_endian::read(field_view.payload().data(), extracted_point[0]);
                microstrain::serialization::big_endian::read(field_view.payload().data(), extracted_point[0]);
                microstrain::serialization::big_endian::read(field_view.payload().data(), extracted_point[0]);

                // TODO: Change to create matrix row vector and add to matrix
                // TODO: Move this to inside loop over discovered points
                double converted_point[3];
                for (uint8_t i = 0; i < 3; ++i)
                {
                    converted_point[i] = static_cast<double>(extracted_point[i]);
                }
            }
        }
    }

    return true;
}
*/

namespace Core
{
    // TODO: Implement
    Eigen::MatrixX3d extractPointMatrixFromRawData(const uint8_t *data, const size_t data_size)
    {
        /*
        // Pass 1: Pre-allocate matrix
        // We aren't working with a device, so the timeouts and timestamp aren't needed.
        // TODO: Setup point estimate to pre-allocate data pointer vector
        // TODO: Change count callback to extract the data pointers to the vector
        // TODO: Use vector size to pre-allocate matrix
        size_t num_points = 0;
        mip::Parser parser(&countPointsInRawData, &num_points, 0);
        parser.parse(data, data_size, 0);
        Eigen::MatrixX3d points_matrix(num_points, 3);

        // TODO: Switch to loop over vector of discovered points instead
        // TODO: Set to extract data from vector of points into matrix
        // Pass 2: Extract points from the binary data into the matrix.
        /*
        for (size_t i = 0; i < num_points; ++i)
        {
           // Get next field
        }
        parser.setCallback<Eigen::MatrixX3d, &extractPointsToMatrix>(points_matrix);

        // TODO: For testing, remove after
        std::cout << "MATRIX:\n";
        std::cout << points_matrix.topRows(10) << std::endl;
        #1#
    */
        return Eigen::MatrixX3d::Zero(data_size, 3);
    }
}
