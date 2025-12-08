# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.6.1
    GIT_SHALLOW TRUE
)
FetchContent_Declare(
    mio
    GIT_REPOSITORY https://github.com/vimpunk/mio.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)
FetchContent_Declare(
    mip_sdk
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/mip_sdk.git
    GIT_TAG develop
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(cli11)
FetchContent_MakeAvailable(mio)
FetchContent_MakeAvailable(mip_sdk)


# ---------------------------------------------------------------
# Application setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

project(offline_mag_cal VERSION ${REPO_NUMERIC_VERSION})
set(PROJECT_VERSION ${REPO_FULL_VERSION})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CORE "offline_mag_cal_core")
set(CLI "offline_mag_cal_cli")

add_library(${CORE})
add_executable(${CLI})

set_target_properties(${CLI}
    PROPERTIES
        OUTPUT_NAME offline-mag-cal
)

target_link_libraries(${CORE}
    Eigen3::Eigen
    microstrain_mag_cal
    ${MIP_LIBRARY}
)
target_link_libraries(${CLI}
    PRIVATE
        CLI11::CLI11
        mio::mio
        offline_mag_cal_core
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(scripts)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

# Install the executable
install(TARGETS ${CLI}
    DESTINATION bin
    COMPONENT application
)
