# ---------------------------------------------------------------
# Project Repo Setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

# Get project version from git as source of truth.
include(cmake/git_version.cmake)
get_project_version_from_git_tag("v" REPO_FULL_VERSION REPO_NUMERIC_VERSION)

project(microstrain_mag_cal VERSION ${REPO_NUMERIC_VERSION})
set(PROJECT_VERSION ${REPO_FULL_VERSION})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# ---------------------------------------------------------------
# Shared Compile Options
# ---------------------------------------------------------------

if(MSVC)
    set(COMPILE_OPTIONS
        "/W4" # Enable warnings
        "/WX" # Warnings as errors
        "/MP" # Multi-process compilation
    )
else()
    set(COMPILE_OPTIONS
        "-Wall"   # Enable warnings
        "-Wextra" # Enable extra warnings
        "-Werror" # Warnings as errors
    )
endif()

# ---------------------------------------------------------------
# Shared Dependencies
# ---------------------------------------------------------------

include(FetchContent)

set(MICROSTRAIN_TEST_FRAMEWORK_CPP "doctest" CACHE STRING "" FORCE)

FetchContent_Declare(
    eigen
    # NOTE: I had to do: `git config --global http.sslBackend schannel` to get the SSL certificates to
    # work in windows. Might be worth looking into whether it's better to do this or update OpenSSL
    # certificates manually.
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.0
    GIT_SHALLOW TRUE
)
FetchContent_Declare(microstrain_test
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/microstrain_test.git
    GIT_TAG v0.2.7
)

FetchContent_MakeAvailable(eigen)
FetchContent_MakeAvailable(microstrain_test)

if (MSVC)
    # Known issue by Eigen team. It's a strict warning for code style and doesn't affect
    # anything functionality-wise.
    target_compile_options(eigen INTERFACE /wd5054)
endif()

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(lib)
add_subdirectory(app)

# ---------------------------------------------------------------
# Packaging
# ---------------------------------------------------------------

set(CPACK_PACKAGE_NAME "microstrain-mag-cal")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "MicroStrain by HBK")
# TODO: Set reusable description for README and here
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Suite of library and tools for magnetometer calibration.")

# Install both components by default
set(CPACK_COMPONENTS_ALL lib app)

if(WIN32)
    set(CPACK_GENERATOR "ZIP")
elseif(UNIX)
    set(CPACK_GENERATOR "TGZ")
endif()

include(CPack)

# ---------------------------------------------------------------
# CI/CD
# ---------------------------------------------------------------

# Convenience target to package the suite.
add_custom_target("package_${PROJECT_NAME}"
    # Want to specify the config on multi-config generators, but not single-config ones.
    COMMAND ${CMAKE_CPACK_COMMAND} $<$<BOOL:$<CONFIG>>:-C> $<$<BOOL:$<CONFIG>>:$<CONFIG>>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Packaging ${PROJECT_NAME}..."
)
