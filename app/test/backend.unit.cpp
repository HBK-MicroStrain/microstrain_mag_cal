#include <backend.hpp>

#include <microstrain_mag_cal/analysis.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include <mip/definitions/data_sensor.hpp>

// This factory class allows us to create controllable and replicable fake binary data.
class BinaryDataBuilder
{
public:
    // Builds a mip packet and inserts it into the fake binary data.
    //
    // Pass in mip fields to include in the packet. For example:
    //     ScaledMag{{ 1.0, 2.0, 3.0 }}, ScaledMag{{ 4.0, 5.0, 6.0 }}, ...
    template<typename... Args>
    void addMipPacket(Args&&... args)
    {
        mip::PacketBuf packet(mip::data_sensor::DESCRIPTOR_SET);
        (packet.addField(std::forward<Args>(args)), ...);
        packet.finalize();

        m_data.insert(m_data.end(), packet.data().begin(), packet.data().end());
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


MICROSTRAIN_TEST_CASE("App_Backend", "Mag_cal_data_can_be_extracted_from_binary_into_a_point_matrix")
{
    BinaryDataBuilder builder = BinaryDataBuilder();
    builder.addMipPacket(
        mip::data_sensor::ScaledMag{{CHECK_POINTS[0], CHECK_POINTS[1], CHECK_POINTS[2]}});
    builder.addMipPacket(
        mip::data_sensor::ScaledAccel{{0.0f, 0.0f, 0.0f}},
        mip::data_sensor::ScaledMag{{CHECK_POINTS[3], CHECK_POINTS[4], CHECK_POINTS[5]}});
    builder.addMipPacket(
        mip::data_sensor::ScaledAccel{{0.0f, 0.0f, 0.0f}},
        mip::data_sensor::ScaledMag{{CHECK_POINTS[6], CHECK_POINTS[7], CHECK_POINTS[8]}},
        mip::data_sensor::ScaledGyro{{0.0f, 0.0f, 0.0f}});
    const microstrain::ConstU8ArrayView data_view = builder.data();

    Eigen::MatrixX3d result = backend::extractPointMatrixFromRawData(data_view);

    REQUIRE(result.rows() == 3);
    REQUIRE(result.cols() == 3);
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

MICROSTRAIN_TEST_CASE("App_Backend", "A_properly_formatted_JSON_model_is_output_given_a_fit_result")
{
    const Eigen::RowVector3d hard_iron_offset(10.0, 11.0, 12.0);
    Eigen::Matrix3d soft_iron_matrix;
    soft_iron_matrix << 1.0, 2.0, 3.0,
                        4.0, 5.0, 6.0,
                        7.0, 8.0, 9.0;
    const microstrain_mag_cal::FitResult fit_result(soft_iron_matrix, hard_iron_offset);

    const nlohmann::json result = backend::convertFitResultToJson(fit_result);

    CHECK(result == nlohmann::json::parse(R"(
      {
        "fitResult": "SUCCEEDED",
        "softIronMatrix": {
          "Sxx": 1.0,
          "Sxy": 2.0,
          "Sxz": 3.0,
          "Syx": 4.0,
          "Syy": 5.0,
          "Syz": 6.0,
          "Szx": 7.0,
          "Szy": 8.0,
          "Szz": 9.0
        },
        "hardIronOffset": {
          "x": 10.0,
          "y": 11.0,
          "z": 12.0
        }
      }
    )"));
}
