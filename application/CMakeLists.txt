# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
    mio
    GIT_REPOSITORY https://github.com/vimpunk/mio.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)
FetchContent_Declare(
    mip_sdk
    GIT_REPOSITORY https://github.com/LORD-MicroStrain/mip_sdk.git
    GIT_TAG latest
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(mio)
FetchContent_MakeAvailable(mip_sdk)

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

add_executable(offline_mag_cal_commandline
    "application.cpp"
)

target_link_libraries(offline_mag_cal_commandline
    PRIVATE
        microstrain_mag_cal
        mio::mio
        ${MIP_LIBRARY}
)

# Copy the test file to the build directory
add_custom_command(
    TARGET offline_mag_cal_commandline POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_LIST_DIR}/test_file_for_offline_mag.bin"
        "$<TARGET_FILE_DIR:offline_mag_cal_commandline>/test_file_for_offline_mag.bin"
)
