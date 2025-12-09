# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

set(BACKEND_UNIT_TESTS "backend_unit_tests")

add_executable(${BACKEND_UNIT_TESTS}
    "backend.unit.cpp"
)

target_link_libraries(${BACKEND_UNIT_TESTS}
    PRIVATE
        microstrain_test::cpp
        ${BACKEND}
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET ${BACKEND_UNIT_TESTS}
    LABELS "unit"
)
