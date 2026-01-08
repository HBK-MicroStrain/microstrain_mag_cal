#include <microstrain_mag_cal_apply/composed_coefficients.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include "clean_data.hpp"
#include "shared_fixtures.hpp"

using namespace microstrain_mag_cal;


namespace fixture
{
    const Eigen::Vector3d applyComposedCorrections(const Eigen::Vector3d &raw_point, const FitResult &combined_fit)
    {
        return combined_fit.soft_iron_matrix * (raw_point - combined_fit.hard_iron_offset.transpose());
    }

    const Eigen::Vector3d applyCorrectionsSequentially(
        const Eigen::Vector3d &raw_point,
        const FitResult &old_fit,
        const FitResult &new_fit)
    {
        const Eigen::Vector3d old_calibrated = old_fit.soft_iron_matrix * (raw_point - old_fit.hard_iron_offset.transpose());

        return new_fit.soft_iron_matrix * (old_calibrated - new_fit.hard_iron_offset.transpose());
    }
}


MICROSTRAIN_TEST_CASE("Lib_Apply", "Applying_composed_corrections_is_equivalent_to_applying_them_sequentially")
{
    FitResult old_fit;
    FitResult new_fit;
    old_fit.soft_iron_matrix <<
         1.12345,  0.01234, -0.12345,
         0.23456,  2.12345,  0.34567,
        -0.45678,  0.56789,  3.12345;
    new_fit.soft_iron_matrix <<
         4.12345,  1.01234, -1.12345,
         1.23456,  5.12345,  1.34567,
        -1.45678,  1.56789,  6.12345;
    old_fit.hard_iron_offset << 1.12345, 2.12345, 3.12345;
    new_fit.hard_iron_offset << 4.12345, 5.12345, 6.12345;
    const Eigen::Vector3d raw_point(10.12345, 20.12345, 30.12345);

    const FitResult composed_fit = composeCorrections(old_fit, new_fit);
    const Eigen::Vector3d result_composed = fixture::applyComposedCorrections(raw_point, composed_fit);
    const Eigen::Vector3d result_sequential = fixture::applyCorrectionsSequentially(raw_point, old_fit, new_fit);

    CHECK(result_composed(0) == doctest::Approx(result_sequential(0)).epsilon(0.0001));
    CHECK(result_composed(1) == doctest::Approx(result_sequential(1)).epsilon(0.0001));
    CHECK(result_composed(2) == doctest::Approx(result_sequential(2)).epsilon(0.0001));
}
