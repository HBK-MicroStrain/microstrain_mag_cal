#include <microstrain_mag_cal/preprocessing.hpp>
#include <microstrain_test/microstrain_test.hpp>


MICROSTRAIN_TEST_CASE("Lib_Preprocessing", "A_non_unique_point_cannot_be_added_to_the_point_manager")
{
    microstrain_mag_cal::VoxelGrid point_grid(1.0);
    microstrain_mag_cal::PointManager sut(point_grid);

    sut.addPoint({1.0f, 2.0f, 3.0f});
    sut.addPoint({1.1f, 2.1f, 3.1f});
    sut.addPoint({1.9f, 2.9f, 3.9f});
    Eigen::MatrixX3d result = sut.getMatrix();

    REQUIRE(result.rows() == 1);
    CHECK(result(0, 0) == doctest::Approx(1.0).epsilon(0.01));
    CHECK(result(0, 1) == doctest::Approx(2.0).epsilon(0.01));
    CHECK(result(0, 2) == doctest::Approx(3.0).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Preprocessing", "A_unique_point_can_be_added_to_the_point_manager")
{
    microstrain_mag_cal::VoxelGrid point_grid(1.0);
    microstrain_mag_cal::PointManager sut(point_grid);

    sut.addPoint({1.0f, 2.0f, 3.0f});
    sut.addPoint({4.0f, 5.0f, 6.0f});
    sut.addPoint({7.0f, 8.0f, 9.0f});
    Eigen::MatrixX3d result = sut.getMatrix();

    REQUIRE(result.rows() == 3);
    CHECK(result(0, 0) == doctest::Approx(1.0).epsilon(0.01));
    CHECK(result(0, 1) == doctest::Approx(2.0).epsilon(0.01));
    CHECK(result(0, 2) == doctest::Approx(3.0).epsilon(0.01));
    CHECK(result(1, 0) == doctest::Approx(4.0).epsilon(0.01));
    CHECK(result(1, 1) == doctest::Approx(5.0).epsilon(0.01));
    CHECK(result(1, 2) == doctest::Approx(6.0).epsilon(0.01));
    CHECK(result(2, 0) == doctest::Approx(7.0).epsilon(0.01));
    CHECK(result(2, 1) == doctest::Approx(8.0).epsilon(0.01));
    CHECK(result(2, 2) == doctest::Approx(9.0).epsilon(0.01));
}

MICROSTRAIN_TEST_CASE("Lib_Preprocessing", "Point_manager_metrics_are_tracked_properly")
{
    microstrain_mag_cal::VoxelGrid point_grid(1.0);
    microstrain_mag_cal::PointManager sut(point_grid);

    sut.addPoint({1.0f, 1.0f, 1.0f});
    sut.addPoint({1.1f, 1.1f, 1.1f});
    sut.addPoint({2.0f, 2.0f, 2.0f});
    sut.addPoint({2.1f, 2.1f, 2.1f});
    sut.addPoint({2.9f, 2.9f, 2.9f});
    sut.addPoint({3.0f, 3.0f, 3.0f});
    sut.addPoint({4.0f, 4.0f, 4.0f});
    sut.addPoint({4.1f, 4.1f, 4.1f});
    sut.addPoint({5.0f, 5.0f, 5.0f});

    CHECK(sut.getNumFilteredPoints() == 5);
    CHECK(sut.getNumPointsSeen() == 9);
}
