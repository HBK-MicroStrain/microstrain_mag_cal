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

        void addBias(const Eigen::RowVector3d &bias)
        {
            m_bias = bias;
        }

        void addUniformScaleFactor(const double scale_factor)
        {
            m_error_matrix.diagonal() = Eigen::Vector3d(scale_factor, scale_factor, scale_factor);
        }

        void addScaleFactor(const Eigen::Vector3d &scale_factor)
        {
            m_error_matrix.diagonal() = scale_factor;
        }

        void addUniformCrossCoupling(const double cross_coupling)
        {
            m_error_matrix.triangularView<Eigen::StrictlyUpper>().setConstant(cross_coupling);
            m_error_matrix.triangularView<Eigen::StrictlyLower>().setConstant(cross_coupling);
        }

        [[nodiscard]] Eigen::MatrixX3d applyError() const
        {
            // -----------------------------------------------------------
            // Single mag cal point real-world physics model:
            // -----------------------------------------------------------
            //    m_r = E * m_t + b, where:
            //
            //        m_r := 3x1 column vector of the raw mag cal reading
            //          E := 3x3 error distortion matrix
            //        m_t := 3x1 column vector of the true mag cal value
            //          b := 3x1 column vector of bias
            //
            // -----------------------------------------------------------
            // To convert the model to our format:
            // -----------------------------------------------------------
            //    m_r^T = (E * m_t + b)^T
            //          = m_t^T * E^T + b^T, where:
            //
            //        m_r^T := 1x3 row vector of raw mag cal data
            //          E^T := Flipped 3x3 error distortion matrix
            //        m_t^T := 1x3 row vector of true mag cal data
            //          b^T := 1x3 row vector of bias
            //
            // -----------------------------------------------------------
            //
            // Now, the data matrix is Nx3, where each row is the 1x3 row vector.
            //
            return (m_clean_data * m_error_matrix.transpose()).rowwise() + m_bias;
        }

        [[nodiscard]] Eigen::MatrixX3d getDistortionMatrix() const
        {
           return m_error_matrix;
        }

    private:
        const Eigen::MatrixX3d m_clean_data;  // Copying is safer here.

        Eigen::RowVector3d m_bias{0.0, 0.0, 0.0};
        Eigen::Matrix<double, 3, 3> m_error_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    };


    /// @brief Applies corrections to the given data.
    ///
    /// @param data Nx3 matrix of data with error to correct
    /// @param error_correction 3x3 error correction matrix
    /// @param bias 1x3 row vector of bias
    ///
    /// @returns Nx3 matrix containing the corrected data
    ///
    Eigen::MatrixX3d applyCorrections(
        const Eigen::MatrixX3d &data,
        const Eigen::Matrix3d &error_correction,
        const Eigen::RowVector3d &bias)
    {
        // ---------------------------------------------------------------
        // Correction equation is the inverse of the distortion equation:
        // ---------------------------------------------------------------
        //    m_t = E^-1 * (m_r - b), where:
        //
        //         m_t := 3x1 column vector of true mag cal data
        //        E^-1 := The 3x3 error correction matrix
        //         m_r := 3x1 column vector of raw mag cal data
        //           b := 3x1 column vector of bias
        //
        // ---------------------------------------------------------------
        // To convert the model to our format:
        // ---------------------------------------------------------------
        //     m_t^T = (E^-1 * (m_r - b))^T
        //           = (m_r - b)^T * (E^-1)^T
        //           = (m_r^T - b^T) * (E^-1)^T, where:
        //
        //            m_t^T := 1x3 row vector of true mag cal data
        //         (E^-1)^T := Flipped 3x3 error correction matrix
        //            m_r^T := 1x3 row vector of raw mag cal data
        //              b^T := 1x3 row vector of bias
        //
        // ---------------------------------------------------------------
        //
        // Now, the data matrix is Nx3, where each row is the 1x3 row vector.
        //
        return (data.rowwise() - bias) * error_correction.transpose();
    }

    /// @brief Convenience wrapper for fit result.
    Eigen::MatrixX3d applyCorrections(const Eigen::MatrixX3d &data, const FitResult &result)
    {
        return applyCorrections(data, result.soft_iron_matrix, result.hard_iron_offset);
    }
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
    fixture::MagCalDataBuilder builder(fixture::CLEAN_DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addUniformScaleFactor(1.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitSphere(data_with_error, field_strength, initial_offset);

    // Proper dimensions
    REQUIRE(result.soft_iron_matrix.rows() == 3);
    REQUIRE(result.soft_iron_matrix.cols() == 3);
    REQUIRE(result.hard_iron_offset.size() == 3);
    // Diagonal (uniform scaling correction)
    constexpr double expected_correction = 1 / 1.5;
    CHECK(result.soft_iron_matrix(0, 0) == doctest::Approx(expected_correction).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 1) == doctest::Approx(expected_correction).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 2) == doctest::Approx(expected_correction).epsilon(0.001));
    // Non-diagonal (no cross-coupling)
    CHECK(result.soft_iron_matrix(0, 1) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 2) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 0) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 2) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 0) == doctest::Approx(0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 1) == doctest::Approx(0).epsilon(0.001));
    // Bias
    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.001));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.001));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("Calibration", "Ellipsoidal_fit_recovers_hard_iron_offset")
{
    fixture::MagCalDataBuilder builder(fixture::CLEAN_DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, field_strength, initial_offset);

    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.01));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.01));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Calibration", "Ellipsoidal_fit_corrects_data_to_sphere_of_reference_field_strength")
{
    fixture::MagCalDataBuilder builder(fixture::CLEAN_DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, field_strength, initial_offset);

    const Eigen::MatrixX3d corrected_data = fixture::applyCorrections(data_with_error, result);
    const Eigen::VectorXd data_field_strengths = corrected_data.rowwise().norm();
    CHECK(data_field_strengths.minCoeff() == doctest::Approx(field_strength).epsilon(0.01));
    CHECK(data_field_strengths.maxCoeff() == doctest::Approx(field_strength).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Calibration", "Ellipsoidal_fit_produces_inverse_of_distortion_matrix")
{
    fixture::MagCalDataBuilder builder(fixture::CLEAN_DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    constexpr double field_strength = 1;
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, field_strength, initial_offset);

    CHECK((result.soft_iron_matrix * builder.getDistortionMatrix() - Eigen::Matrix3d::Identity()).norm() < 0.01);
}
