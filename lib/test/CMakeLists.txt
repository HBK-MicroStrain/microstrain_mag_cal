# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

set(LIB_UNIT_TESTS "lib_unit_tests")

add_executable(${LIB_UNIT_TESTS}
    "analysis.unit.cpp"
    "calibration.unit.cpp"
    #"combined_coefficients.unit.cpp"
    #"preprocessing.unit.cpp"
)

target_link_libraries(${LIB_UNIT_TESTS}
    PRIVATE
        microstrain_test::cpp
        ${LIB_CALIBRATION}
        ${LIB_APPLY_CALIBRATION}
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET ${LIB_UNIT_TESTS}
    LABELS "unit"
)
