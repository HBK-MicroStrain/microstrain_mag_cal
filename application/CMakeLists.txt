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
    eigen
    # NOTE: I had to do: `git config --global http.sslBackend schannel` to get the SSL certificates to
    # work on windows. Might be worth looking into whether it's better to do this or update OpenSSL
    # certificates manually.
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.0
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
FetchContent_MakeAvailable(eigen)
FetchContent_MakeAvailable(mio)
FetchContent_MakeAvailable(mip_sdk)

if (MSVC)
    # Known issue by Eigen team. It's a strict warning for code style and doesn't affect
    # anything functionality-wise.
    target_compile_options(eigen INTERFACE /wd5054)
endif()

# ---------------------------------------------------------------
# Application setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

project(offline_mag_cal
    VERSION 0.0.0
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(offline_mag_cal_core)
add_executable(offline_mag_cal_cli)

set_target_properties(offline_mag_cal_cli
    PROPERTIES
        OUTPUT_NAME offline-mag-cal
)

target_link_libraries(offline_mag_cal_core
    Eigen3::Eigen
    microstrain_mag_cal
    ${MIP_LIBRARY}
)
target_link_libraries(offline_mag_cal_cli
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
