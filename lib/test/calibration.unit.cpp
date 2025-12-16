#include <microstrain_mag_cal/calibration.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include "clean_data.hpp"
#include "shared_fixtures.hpp"

using namespace microstrain_mag_cal;


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


MICROSTRAIN_TEST_CASE("Lib_Calibration", "The_initial_hard_iron_offset_estimate_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);

    Eigen::RowVector3d result = estimateInitialHardIronOffset(empty_matrix);

    REQUIRE(result.cols() == 3);
    CHECK(result(0) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result(1) == doctest::Approx(0.0).epsilon(0.001));
    CHECK(result(2) == doctest::Approx(0.0).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "The_initial_hard_iron_offset_estimate_is_near_the_data_center")
{
    Eigen::RowVector3d result = estimateInitialHardIronOffset(CHECK_POINTS);

    REQUIRE(result.cols() == 3);
    CHECK(result(0) == doctest::Approx(-0.161364).epsilon(0.001));
    CHECK(result(1) == doctest::Approx(0.081262).epsilon(0.001));
    CHECK(result(2) == doctest::Approx(0.240466).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Measured_field_strength_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(empty_matrix);

    const double result = calculateMeanMeasuredFieldStrength(empty_matrix, initial_offset);

    CHECK(result == 0.0);
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Measured_field_strength_matches_Inertial_connect")
{
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(CHECK_POINTS);

    const double result = calculateMeanMeasuredFieldStrength(CHECK_POINTS, initial_offset);

    CHECK(result == doctest::Approx(0.383).epsilon(0.001));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Spherical_fit_recovers_hard_iron_offset")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addUniformScaleFactor(1.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitSphere(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.01));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.01));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Spherical_fit_corrects_data_to_sphere_of_reference_field_strength")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addUniformScaleFactor(1.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitSphere(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    const Eigen::MatrixX3d corrected_data = fixture::applyCorrections(data_with_error, result);
    const Eigen::VectorXd data_field_strengths = corrected_data.rowwise().norm();
    CHECK(data_field_strengths.minCoeff() == doctest::Approx(fixture::clean_data::FIELD_STRENGTH).epsilon(0.01));
    CHECK(data_field_strengths.maxCoeff() == doctest::Approx(fixture::clean_data::FIELD_STRENGTH).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Spherical_fit_produces_inverse_of_distortion_matrix")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addUniformScaleFactor(1.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitSphere(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK((result.soft_iron_matrix * builder.getDistortionMatrix()).isApprox(Eigen::Matrix3d::Identity(), 0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Spherical_fit_corrected_data_requires_no_further_correction")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addUniformScaleFactor(1.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult initial_fit = fitSphere(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);
    const Eigen::MatrixX3d corrected_data = fixture::applyCorrections(data_with_error, initial_fit);
    const FitResult refined_fit = fitSphere(corrected_data, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK_MESSAGE(refined_fit.soft_iron_matrix.isApprox(Eigen::Matrix3d::Identity(), 0.01), refined_fit.soft_iron_matrix);
    CHECK_MESSAGE(refined_fit.hard_iron_offset.isApprox(Eigen::RowVector3d::Zero(), 0.01), refined_fit.hard_iron_offset);
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Ellipsoidal_fit_recovers_hard_iron_offset")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK(result.hard_iron_offset(0) == doctest::Approx(2.1).epsilon(0.01));
    CHECK(result.hard_iron_offset(1) == doctest::Approx(2.2).epsilon(0.01));
    CHECK(result.hard_iron_offset(2) == doctest::Approx(2.3).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Ellipsoidal_fit_corrects_data_to_sphere_of_reference_field_strength")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    const Eigen::MatrixX3d corrected_data = fixture::applyCorrections(data_with_error, result);
    const Eigen::VectorXd data_field_strengths = corrected_data.rowwise().norm();
    CHECK(data_field_strengths.minCoeff() == doctest::Approx(fixture::clean_data::FIELD_STRENGTH).epsilon(0.01));
    CHECK(data_field_strengths.maxCoeff() == doctest::Approx(fixture::clean_data::FIELD_STRENGTH).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "Ellipsoidal_fit_produces_inverse_of_distortion_matrix")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult result = fitEllipsoid(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK((result.soft_iron_matrix * builder.getDistortionMatrix()).isApprox(Eigen::Matrix3d::Identity(), 0.01));
}

// TODO: Maybe integration test - runs a little slow
MICROSTRAIN_TEST_CASE("Lib_Calibration", "Ellipsoidal_fit_corrected_data_requires_no_further_correction")
{
    fixture::MagCalDataBuilder builder(fixture::clean_data::DATA);
    builder.addBias({2.1, 2.2, 2.3});
    builder.addScaleFactor({1.1, 2.2, 3.3});
    builder.addUniformCrossCoupling(0.5);
    const Eigen::MatrixX3d data_with_error = builder.applyError();
    const Eigen::RowVector3d initial_offset = estimateInitialHardIronOffset(data_with_error);

    const FitResult initial_fit = fitEllipsoid(data_with_error, fixture::clean_data::FIELD_STRENGTH, initial_offset);
    const Eigen::MatrixX3d corrected_data = fixture::applyCorrections(data_with_error, initial_fit);
    const FitResult refined_fit = fitEllipsoid(corrected_data, fixture::clean_data::FIELD_STRENGTH, initial_offset);

    CHECK_MESSAGE(refined_fit.soft_iron_matrix.isApprox(Eigen::Matrix3d::Identity(), 0.01), refined_fit.soft_iron_matrix);
    CHECK_MESSAGE(refined_fit.hard_iron_offset.isApprox(Eigen::RowVector3d::Zero(), 0.01), refined_fit.hard_iron_offset);
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "A_properly_formatted_JSON_model_is_output_given_a_fit_result")
{
    const Eigen::RowVector3d hard_iron_offset(10.0, 11.0, 12.0);
    Eigen::Matrix3d soft_iron_matrix;
    soft_iron_matrix << 1.0, 2.0, 3.0,
                        4.0, 5.0, 6.0,
                        7.0, 8.0, 9.0;
    const FitResult fit_result(soft_iron_matrix, hard_iron_offset);

    const nlohmann::json result = convertFitResultToJson(fit_result);

    CHECK(result == nlohmann::json::parse(R"(
      {
        "error": "NONE",
        "softIronMatrix": {
          "xx": 1.0,
          "xy": 2.0,
          "xz": 3.0,
          "yx": 4.0,
          "yy": 5.0,
          "yz": 6.0,
          "zx": 7.0,
          "zy": 8.0,
          "zz": 9.0
        },
        "hardIronOffset": {
          "x": 10.0,
          "y": 11.0,
          "z": 12.0
        }
      }
    )"));
}

MICROSTRAIN_TEST_CASE("Lib_Calibration", "The_calibration_coefficients_can_be_parsed_from_JSON")
{
    nlohmann::json input_json = nlohmann::json::parse(R"(
      {
        "error": "NONE",
        "softIronMatrix": {
          "xx": 1.0,
          "xy": 2.0,
          "xz": 3.0,
          "yx": 4.0,
          "yy": 5.0,
          "yz": 6.0,
          "zx": 7.0,
          "zy": 8.0,
          "zz": 9.0
        },
        "hardIronOffset": {
          "x": 10.0,
          "y": 11.0,
          "z": 12.0
        }
      }
    )");

    const FitResult result = parseCalibrationFromJson(input_json);

    Eigen::Matrix3d soft_iron_matrix;
    soft_iron_matrix << 1.0, 2.0, 3.0,
                        4.0, 5.0, 6.0,
                        7.0, 8.0, 9.0;
    Eigen::RowVector3d hard_iron_offset(10.0, 11.0, 12.0);

    CHECK(result.error == FitResult::Error::NONE);
    CHECK_MESSAGE(result.soft_iron_matrix.isApprox(soft_iron_matrix, 0.01), result.soft_iron_matrix);
    CHECK_MESSAGE(result.hard_iron_offset.isApprox(hard_iron_offset, 0.01), result.hard_iron_offset);
}
