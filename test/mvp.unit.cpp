#include <microstrain_mag_cal.hpp>
#include <microstrain_test/microstrain_test.hpp>

// Data points taken from a real InertialConnect data capture. All expected values for tests are
// taken from InertialConnect based off this input data.
constexpr std::array<double, 20 * 3> raw_points = {
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
const Eigen::Matrix<double, 20, 3, Eigen::RowMajor> CHECK_POINTS(raw_points.data());


MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_handles_no_data_points")
{
    Eigen::MatrixX3d empty_matrix;
    empty_matrix.resize(0, 3);

    const double result = MicrostrainMagCal::calculate_measured_field_strength(empty_matrix);

    CHECK(result == 0.0);
}

MICROSTRAIN_TEST_CASE("MVP", "Measured_field_strength_matches_Inertial_connect")
{
    const double result = MicrostrainMagCal::calculate_measured_field_strength(CHECK_POINTS);

    CHECK(result == doctest::Approx(0.383).epsilon(0.001));
}
