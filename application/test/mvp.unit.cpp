#include <mag_cal_core.hpp>

#include <microstrain_test/microstrain_test.hpp>
#include "mip/definitions/data_sensor.hpp"


// This factory class allows us to create controllable and replicable fake binary data.
class BinaryDataBuilder
{
public:
    // Builds a mip packet and inserts it into the fake binary data.
    //
    // Pass in mip fields to include in the packet. For example:
    //     ScaledMag{{ 1.0, 2.0, 3.0 }}, ScaledMag{{ 4.0, 5.0, 6.0 }}, ...
    template<typename... Args>
    BinaryDataBuilder& addMipPacket(Args&&... args)
    {
        mip::PacketBuf packet(mip::data_sensor::DESCRIPTOR_SET);
        (packet.addField(std::forward<Args>(args)), ...);
        packet.finalize();

        m_data.insert(m_data.end(), packet.data().begin(), packet.data().end());

        return *this;
    }

    microstrain::ConstU8ArrayView data() const
    {
        return {m_data.data(), m_data.size()};
    }

private:
    std::vector<uint8_t> m_data;
};

std::array CHECK_POINTS {
     1.123456789f, -2.123456789f,  3.123456789f,
    -4.123456789f,  5.123456789f, -6.123456789f,
     7.123456789f, -8.123456789f,  9.123456789f
};


MICROSTRAIN_TEST_CASE("MVP", "Mag_cal_data_can_be_extracted_from_binary_into_a_point_matrix")
{
    BinaryDataBuilder builder = BinaryDataBuilder()
        .addMipPacket(
            mip::data_sensor::ScaledMag{{CHECK_POINTS[0], CHECK_POINTS[1], CHECK_POINTS[2]}}
        )
        .addMipPacket(
            mip::data_sensor::ScaledAccel{{0.0f, 0.0f, 0.0f}}, // Noise
            mip::data_sensor::ScaledMag{{CHECK_POINTS[3], CHECK_POINTS[4], CHECK_POINTS[5]}}
        )
        .addMipPacket(
            mip::data_sensor::ScaledAccel{{0.0f, 0.0f, 0.0f}}, // Noise
            mip::data_sensor::ScaledMag{{CHECK_POINTS[6], CHECK_POINTS[7], CHECK_POINTS[8]}},
            mip::data_sensor::ScaledGyro{{0.0f, 0.0f, 0.0f}}    // Noise
        );
    const microstrain::ConstU8ArrayView data_view = builder.data();

    Eigen::MatrixX3d result = mag_cal_core::extractPointMatrixFromRawData(data_view);

    // TODO: Add check for size

    CHECK(result(0, 0) == doctest::Approx(CHECK_POINTS[0]).epsilon(0.001));
    CHECK(result(0, 1) == doctest::Approx(CHECK_POINTS[1]).epsilon(0.001));
    CHECK(result(0, 2) == doctest::Approx(CHECK_POINTS[2]).epsilon(0.001));
    CHECK(result(1, 0) == doctest::Approx(CHECK_POINTS[3]).epsilon(0.001));
    CHECK(result(1, 1) == doctest::Approx(CHECK_POINTS[4]).epsilon(0.001));
    CHECK(result(1, 2) == doctest::Approx(CHECK_POINTS[5]).epsilon(0.001));
    CHECK(result(2, 0) == doctest::Approx(CHECK_POINTS[6]).epsilon(0.001));
    CHECK(result(2, 1) == doctest::Approx(CHECK_POINTS[7]).epsilon(0.001));
    CHECK(result(2, 2) == doctest::Approx(CHECK_POINTS[8]).epsilon(0.001));
}
