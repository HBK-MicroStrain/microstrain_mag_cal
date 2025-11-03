#include <core.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include <mip/mip_packet.h>

class BinaryDataBuilder
{
public:
    BinaryDataBuilder& addMagCalPointVector(const float x, const float y, const float z)
    {
        mip::C::mip_packet_view packet;
        uint8_t buffer[mip::C::MIP_PACKET_LENGTH_MAX];
        mip::C::mip_packet_create(&packet, buffer, sizeof(buffer), ScaledMag::DESCRIPTOR_SET);

        uint8_t payload[3 * sizeof(float)];
        std::memcpy(payload, &x, sizeof(float));
        std::memcpy(payload + sizeof(float), &y, sizeof(float));
        std::memcpy(payload + 2 * sizeof(float), &z, sizeof(float));

        mip::C::mip_packet_add_field(&packet, ScaledMag::FIELD_DESCRIPTOR, payload, sizeof(payload));
        mip::C::mip_packet_finalize(&packet);

        const uint8_t *packet_data = mip::C::mip_packet_buffer(&packet);
        const uint8_t packet_length = mip::C::mip_packet_total_length(&packet);

        m_data.insert(m_data.end(), packet_data, packet_data + packet_length);

        return *this;
    }

private:
    std::vector<uint8_t> m_data;
};

// TODO: Update with proper output
// Data points taken from a real InertialConnect binary recording. All expected values for tests are
// taken from InertialConnect based off this input data.
static constexpr std::array<double, 20 * 3> raw_points = {
    /*  1 */ 0.177713 ,  0.0808103382, -0.213378355 ,
};
static const Eigen::Matrix<double, 20, 3, Eigen::RowMajor> CHECK_POINTS(raw_points.data());


MICROSTRAIN_TEST_CASE("MVP", "Extracting_a_points_matrix_contains_all_points_from_the_raw_data")
{
    // TODO: Implement
}
