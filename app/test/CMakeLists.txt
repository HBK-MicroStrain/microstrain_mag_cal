# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

set(APP_UNIT_TESTS "app_unit_tests")

add_executable(${APP_UNIT_TESTS}
    "app.unit.cpp"
)

target_link_libraries(${APP_UNIT_TESTS}
    PRIVATE
        microstrain_test::cpp
        ${APP}
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET ${APP_UNIT_TESTS}
    LABELS "unit"
)
