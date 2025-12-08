# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

set(UNIT_TESTS "mag_cal_core_unit_tests")

add_executable(${UNIT_TESTS}
    "analysis.unit.cpp"
    "calibration.unit.cpp"
    "preprocessing.unit.cpp"
)

target_link_libraries(${UNIT_TESTS} PRIVATE
    microstrain_test::cpp
    ${LIB_CORE}
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET ${UNIT_TESTS}
    LABELS "unit"
)
