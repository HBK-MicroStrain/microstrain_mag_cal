#include <microstrain_mag_cal_apply/combined_coefficients.hpp>
#include <microstrain_test/microstrain_test.hpp>
#include "clean_data.hpp"
#include "shared_fixtures.hpp"

using namespace microstrain_mag_cal;


const Eigen::Vector3d applyCombinedCorrections(const Eigen::RowVector3d &raw_point, const FitResult &combined_fit)
{
    return combined_fit.soft_iron_matrix * (raw_point - combined_fit.hard_iron_offset);
}

const Eigen::Vector3d applyCorrectionsSequentially(
    const Eigen::RowVector3d &raw_point,
    const FitResult &old_fit,
    const FitResult &new_fit)
{
    const Eigen::RowVector3d old_calibrated = old_fit.soft_iron_matrix * (raw_point - old_fit.hard_iron_offset);

    return new_fit.soft_iron_matrix * (old_calibrated - new_fit.hard_iron_offset);
}


MICROSTRAIN_TEST_CASE("Lib_Apply", "Applying_combined_corrections_is_equivalent_to_applying_them_sequentially")
{
    FitResult old_fit;
    FitResult new_fit;
    old_fit.soft_iron_matrix <<
         1.21213, 0.01196, -0.05057,
         0.01196, 1.35210,  0.06738,
        -0.05057, 0.06738,  1.34479;
    new_fit.soft_iron_matrix <<
         2.21213, 1.01196, -3.05057,
         8.01196, 7.35210,  3.06738,
        -1.05057, 2.06738,  1.34479;
    old_fit.hard_iron_offset << 0.00426, 0.10610, 0.17490;
    new_fit.hard_iron_offset << 1.00426, 2.10610, 3.17490;

    const FitResult combined_fit = combineCorrections(old_fit, new_fit);

    const Eigen::RowVector3d raw_point(1.0, 2.0, 3.0);
    const Eigen::Vector3d result_combined = applyCombinedCorrections(raw_point, combined_fit);
    const Eigen::Vector3d result_sequential = applyCorrectionsSequentially(raw_point, old_fit, new_fit);
    CHECK(result_combined(0) == result_sequential(0));
    CHECK(result_combined(1) == result_sequential(1));
    CHECK(result_combined(2) == result_sequential(2));
}
