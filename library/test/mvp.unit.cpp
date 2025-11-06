#include <microstrain_mag_cal.hpp>
#include <microstrain_test/microstrain_test.hpp>


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


MICROSTRAIN_TEST_CASE("MVP", "The_initial_hard_iron_offset_is_near_the_data_center")
{
    Eigen::RowVector3d result = microstrain_mag_cal::estimateInitialHardIronOffset(CHECK_POINTS);

    REQUIRE(result.cols() == 3);
    CHECK(result(0) == doctest::Approx(-0.161364).epsilon(0.001));
    CHECK(result(1) == doctest::Approx(0.081262).epsilon(0.001));
    CHECK(result(2) == doctest::Approx(0.240466).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);

    const double result = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(empty_matrix);

    CHECK(result == 0.0);
}

MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_matches_Inertial_connect")
{
    const double result = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(CHECK_POINTS);

    CHECK(result == doctest::Approx(0.383).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Spatial_coverage_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);

    const double result = microstrain_mag_cal::calculateSpatialCoverage(empty_matrix);

    CHECK(result == 0.0);
}

MICROSTRAIN_TEST_CASE("MVP", "Spatial_coverage_matches_InertialConnect")
{
    const double result = microstrain_mag_cal::calculateSpatialCoverage(CHECK_POINTS);

    CHECK(result == doctest::Approx(3.125).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Spherical_fit_matches_Inertial_connect")
{
    constexpr double field_strength = 0.557;

    const microstrain_mag_cal::FitResult result = microstrain_mag_cal::calculateSphericalFit(CHECK_POINTS, field_strength);

    CHECK(result.soft_iron_matrix(0, 0) == doctest::Approx(0.764696).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 1) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 2) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 0) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 1) == doctest::Approx(0.764696).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 2) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 0) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 1) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 2) == doctest::Approx(0.764696).epsilon(0.001));

    CHECK(result.hard_iron_offset(0) == doctest::Approx(-0.01354).epsilon(0.001));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(0.09346).epsilon(0.001));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(0.15742).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Ellipsoidal_fit_matches_Inertial_connect")
{
    constexpr double field_strength = 0.557;

    const microstrain_mag_cal::FitResult result = microstrain_mag_cal::calculateEllipsoidalFit(CHECK_POINTS, field_strength);

    REQUIRE(result.soft_iron_matrix.rows() == 3);
    REQUIRE(result.soft_iron_matrix.cols() == 3);

    CHECK(result.soft_iron_matrix(0, 0) == doctest::Approx(1.21213).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 1) == doctest::Approx(0.01196).epsilon(0.001));
    CHECK(result.soft_iron_matrix(0, 2) == doctest::Approx(-0.05057).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 0) == doctest::Approx(0.01196).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 1) == doctest::Approx(1.35210).epsilon(0.001));
    CHECK(result.soft_iron_matrix(1, 2) == doctest::Approx(0.06738).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 0) == doctest::Approx(-0.05057).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 1) == doctest::Approx(0.06738).epsilon(0.001));
    CHECK(result.soft_iron_matrix(2, 2) == doctest::Approx(1.34479).epsilon(0.001));

    CHECK(result.hard_iron_offset(0) == doctest::Approx(0.00426).epsilon(0.001));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(0.10610).epsilon(0.001));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(0.17490).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("MVP", "Fit_RMSE_matches_InertialConnect")
{
    Eigen::Matrix<double, 3, 3> soft_iron_matrix;
    soft_iron_matrix <<
         1.21213, 0.01196, -0.05057,
         0.01196, 1.35210,  0.06738,
        -0.05057, 0.06738,  1.34479;
    const Eigen::Vector3d hard_iron_offset(0.00426, 0.10610, 0.17490);
    const microstrain_mag_cal::FitResult fit_result(soft_iron_matrix, hard_iron_offset, true);
    constexpr double field_strength = 0.557;

    const double result = microstrain_mag_cal::calculateFitRMSE(CHECK_POINTS, fit_result, field_strength);

    CHECK(result == doctest::Approx(0.010).epsilon(0.001));
}
