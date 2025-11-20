#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include "clean_data.hpp"

using namespace microstrain_mag_cal;


namespace fixture
{
    /// @brief Creates a matrix of synthetic magnetometer data with the given error coefficients.
    ///
    /// The matrix is created using the clean data matrix generated from the script.
    ///
    /// Reference: https://pmc.ncbi.nlm.nih.gov/articles/PMC8401862/#sec2-sensors-21-05288
    ///
    /// Usage:
    /// 1) Add any desired error coefficients
    /// 2) Apply the error (returns the matrix)
    ///
    class MagCalDataBuilder
    {
    public:
        explicit MagCalDataBuilder(const Eigen::MatrixX3d &clean_data) : m_clean_data(clean_data) {}

        MagCalDataBuilder addBias(const Eigen::RowVector3d &bias)
        {
            m_bias = bias;

            return *this;
        }

        MagCalDataBuilder addUniformScaleFactor(const double scale_factor)
        {
            m_error_matrix.diagonal() = Eigen::Vector3d(scale_factor, scale_factor, scale_factor);

            return *this;
        }

        MagCalDataBuilder addScaleFactor(const Eigen::Vector3d &scale_factor)
        {
            m_error_matrix.diagonal() = scale_factor;

            return *this;
        }

        MagCalDataBuilder addUniformCrossCoupling(const double cross_coupling)
        {
            m_error_matrix.triangularView<Eigen::StrictlyUpper>().setConstant(cross_coupling);
            m_error_matrix.triangularView<Eigen::StrictlyLower>().setConstant(cross_coupling);

            return *this;
        }

        [[nodiscard]] Eigen::MatrixX3d applyError() const
        {
            return (m_clean_data * m_error_matrix).rowwise() + m_bias;
        }

    private:
        const Eigen::MatrixX3d& m_clean_data;

        Eigen::RowVector3d m_bias{0.0, 0.0, 0.0};
        Eigen::Matrix<double, 3, 3> m_error_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    };
}


// Data points taken from a real InertialConnect data capture. All expected values for tests are
// taken from InertialConnect based off this input data.
static constexpr std::array<double, 20 * 3> raw_points = {
    /*  1 */ -0.159911692 ,  0.0808103382, -0.213378355 ,
    /*  2 */ -0.357420951 ,  0.0722654387, -0.131103173 ,
    /*  3 */ -0.433592647 ,  0.0410155207,  0.0539549403,
    /*  4 */ -0.37255764  ,  0.0529783815,  0.397948205 ,
    /*  5 */ -0.185058117 ,  0.161620677 ,  0.53173691  ,
    /*  6 */ -0.170409724 ,  0.187499523 ,  0.531004488 ,
    /*  7 */ -0.0598143004,  0.304686725 ,  0.506346345 ,
    /*  8 */ -0.0280760992,  0.317382008 ,  0.500242829 ,
    /*  9 */ -0.0241698604,  0.521238923 ,  0.164062083 ,
    /* 10 */ -0.0346678793,  0.520506501 ,  0.137206674 ,
    /* 11 */ -0.25610286  ,  0.248778656 , -0.162597239 ,
    /* 12 */ -0.273436785 ,  0.228270903 , -0.161132395 ,
    /* 13 */ -0.373778343 , -0.0949704572,  0.0732420012,
    /* 14 */ -0.423338771 ,  0.0288085192,  0.225097075 ,
    /* 15 */ -0.281981707 , -0.102538802 ,  0.423827052 ,
    /* 16 */ -0.0854490026, -0.271239549 ,  0.343504965 ,
    /* 17 */ -0.0722654387, -0.284911364 ,  0.320555806 ,
    /* 18 */  0.198241681 , -0.262938768 ,  0.133300439 ,
    /* 19 */  0.0866696984, -0.0373534188,  0.579832494 ,
    /* 20 */  0.0798337832, -0.0866696984,  0.555662632
};
static const Eigen::Matrix<double, 20, 3, Eigen::RowMajor> CHECK_POINTS(raw_points.data());


MICROSTRAIN_TEST_CASE("MVP", "The_initial_hard_iron_offset_estimate_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);

    Eigen::RowVector3d result = estimateInitialHardIronOffset(empty_matrix);

    REQUIRE(result.cols() == 3);
    CHECK(result(0) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result(1) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result(2) == doctest::Approx(0.0).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "The_initial_hard_iron_offset_estimate_is_near_the_data_center")
{
    Eigen::RowVector3d result = estimateInitialHardIronOffset(CHECK_POINTS);

    REQUIRE(result.cols() == 3);
    CHECK(result(0) == doctest::Approx(-0.161364).epsilon(0.001));
    CHECK(result(1) == doctest::Approx(0.081262).epsilon(0.001));
    CHECK(result(2) == doctest::Approx(0.240466).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(empty_matrix);

    const double result = calculateMeanMeasuredFieldStrength(empty_matrix, initial_offset);

    CHECK(result == 0.0);
}

MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_matches_Inertial_connect")
{
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(CHECK_POINTS);

    const double result = calculateMeanMeasuredFieldStrength(CHECK_POINTS, initial_offset);

    CHECK(result == doctest::Approx(0.383).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("Calibration", "Spherical_fit_provides_correction_matrix")
{
    const Eigen::MatrixX3d data_with_error = fixture::MagCalDataBuilder(fixture::CLEAN_DATA)
        .addBias({2.1, 2.2, 2.3})
        .addUniformScaleFactor(1.5)
        .applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitSphere(data_with_error, field_strength, initial_offset);

    /*
    REQUIRE(result.soft_iron_matrix.rows() == 3);
    REQUIRE(result.soft_iron_matrix.cols() == 3);
    REQUIRE(result.hard_iron_offset.size() == 3);
    // Soft iron should be diagonal (no cross-coupling)
    CHECK(result.soft_iron_matrix(0, 1) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 2) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 0) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 2) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 0) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 1) == doctest::Approx(0).epsilon(0.001));
    // Diagonal elements should all be equal (uniform scaling)
    CHECK(result.soft_iron_matrix(0, 0) == doctest::Approx(result.soft_iron_matrix(1, 1)).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 1) == doctest::Approx(result.soft_iron_matrix(2, 2)).epsilon(0.001));
    // Hard-iron should be recovered correctly
    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.001));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.001));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.001));
*/
}

MICROSTRAIN_TEST_CASE("MVP", "Ellipsoidal_fit_matches_Inertial_connect")
{
    const Eigen::MatrixX3d data_with_error = fixture::MagCalDataBuilder(fixture::CLEAN_DATA)
        .addBias({2.1, 2.2, 2.3})
        .addScaleFactor({1.1, 2.2, 3.3})
        .addUniformCrossCoupling(0.5)
        .applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, field_strength, initial_offset);

    /*
    REQUIRE(result.soft_iron_matrix.rows() == 3);
    REQUIRE(result.soft_iron_matrix.cols() == 3);

    CHECK(result.soft_iron_matrix(0, 0) == doctest::Approx(1.1).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 1) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 2) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 0) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 1) == doctest::Approx(2.2).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 2) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 0) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 1) == doctest::Approx(0.5).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 2) == doctest::Approx(3.3).epsilon(0.001));

    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.001));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.001));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.001));
*/
}
