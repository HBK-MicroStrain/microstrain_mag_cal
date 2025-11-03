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
        const uint_least16_t packet_length = mip::C::mip_packet_total_length(&packet);

        m_data.insert(m_data.end(), packet_data, packet_data + packet_length);

        return *this;
    }

    BinaryDataBuilder& addNoise(const std::initializer_list<uint8_t> noise)
    {
        m_data.insert(m_data.end(), noise.begin(), noise.end());

        return *this;
    }

    const uint8_t* data() const
    {
        return m_data.data();
    }

    size_t data_size() const
    {
        return m_data.size();
    }

private:
    std::vector<uint8_t> m_data;
};

static constexpr std::array<double, 3 * 3> raw_points = {
      1.123456789 ,  -2.123456789,   3.123456789,
     -4.123456789 ,   5.123456789,  -6.123456789,
      7.123456789 ,  -8.123456789,   9.123456789,
};
static const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> CHECK_POINTS(raw_points.data());


MICROSTRAIN_TEST_CASE("MVP", "Extracting_a_points_matrix_contains_all_points_from_the_raw_data")
{
    BinaryDataBuilder builder = BinaryDataBuilder()
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addMagCalPointVector(1.123456789f,  -2.123456789f,   3.123456789f)
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addMagCalPointVector(-4.123456789f,   5.123456789f,  -6.123456789f)
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF})
        .addMagCalPointVector(7.123456789f,  -8.123456789f,   9.123456789f)
        .addNoise({0xFF, 0xAB, 0xCD, 0xEF});
    const uint8_t *data = builder.data();
    const size_t data_size = builder.data_size();

    Eigen::MatrixX3d result = Core::extractPointMatrixFromRawData(data, data_size);

    CHECK(result(0, 0) == doctest::Approx(1.123456789).epsilon(0.001));
    CHECK(result(0, 1) == doctest::Approx(-2.123456789).epsilon(0.001));
    CHECK(result(0, 2) == doctest::Approx(3.123456789).epsilon(0.001));
    CHECK(result(1, 0) == doctest::Approx(-4.123456789).epsilon(0.001));
    CHECK(result(1, 1) == doctest::Approx(5.123456789).epsilon(0.001));
    CHECK(result(1, 2) == doctest::Approx(-6.123456789).epsilon(0.001));
    CHECK(result(2, 0) == doctest::Approx(7.123456789).epsilon(0.001));
    CHECK(result(2, 1) == doctest::Approx(-8.123456789).epsilon(0.001));
    CHECK(result(2, 2) == doctest::Approx(9.123456789).epsilon(0.001));
}
