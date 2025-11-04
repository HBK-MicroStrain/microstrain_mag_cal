#include <mag_cal_core.hpp>

#include <microstrain_test/microstrain_test.hpp>
#include "mip/definitions/data_sensor.hpp"


// This factory class allows us to create controllable and replicable fake binary data.
class BinaryDataBuilder
{
public:
    BinaryDataBuilder& addMagCalPointVector(const float x, const float y, const float z)
    {
        const mip::PacketBuf packet(mip::data_sensor::ScaledMag{{x, y, z}});
        m_data.insert(m_data.end(), packet.data().begin(), packet.data().end());

        return *this;
    }

    // Adds noise to simulate other data in a binary file.
    //
    // We don't care what that data is, but it's useful for tests to make sure only the correct
    // data is extracted, and that all the correct data is extracted. In other words, it removes
    // potential false negatives that could come from the logic working on just the data to be
    // extracted, but breaking if there is other data in the input.
    BinaryDataBuilder& addNoise(const std::initializer_list<uint8_t> noise)
    {
        m_data.insert(m_data.end(), noise.begin(), noise.end());

        return *this;
    }

    microstrain::ConstU8ArrayView data() const
    {
        return {m_data.data(), m_data.size()};
    }

private:
    std::vector<uint8_t> m_data;
};


MICROSTRAIN_TEST_CASE("MVP", "Extracting_a_point_matrix_contains_all_points_from_the_input_binary_data")
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
    const microstrain::ConstU8ArrayView data_view = builder.data();

    Eigen::MatrixX3d result = mag_cal_core::extractPointMatrixFromRawData(data_view);

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
