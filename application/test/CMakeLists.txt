# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

set(MICROSTRAIN_TEST_FRAMEWORK_CPP "doctest" CACHE STRING "" FORCE)

FetchContent_Declare(microstrain_test
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/microstrain_test.git
    GIT_TAG v0.2.7
)

FetchContent_MakeAvailable(microstrain_test)

# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

add_executable(application_unit_tests
    "app_core.unit.cpp"
)

target_link_libraries(application_unit_tests
    PRIVATE
        microstrain_test::cpp
        offline_mag_cal_core
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET application_unit_tests
    LABELS "unit"
)
