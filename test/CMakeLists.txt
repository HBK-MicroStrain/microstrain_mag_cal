# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------
# TODO: Will be moved to separate repo
set(MICROSTRAIN_TEST_USE_DOCTEST ON CACHE BOOL "" FORCE)
add_subdirectory("microstrain_test")

# ---------------------------------------------------------------
# Test setup
# ---------------------------------------------------------------

add_executable(unit_tests)

target_link_libraries(unit_tests PRIVATE
    microstrain_test::cpp
)

# ---------------------------------------------------------------
# Test discovery
# ---------------------------------------------------------------

include(${microstrain_test_SOURCE_DIR}/scripts/DiscoverTests.cmake)

microstrain_discover_tests_cpp(
    TARGET unit_tests
    LABELS "unit"
)
