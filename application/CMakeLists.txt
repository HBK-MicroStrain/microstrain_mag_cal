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

FetchContent_MakeAvailable(mio)

# ---------------------------------------------------------------
# Application setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

project(offline_mag_cal
    VERSION 0.0.0
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(offline_mag_cal_commandline
    "application.cpp"
)

target_link_libraries(offline_mag_cal_commandline
    PRIVATE
        mio::mio
        microstrain_mag_cal
)

# Copy the test file to the build directory
add_custom_command(
    TARGET offline_mag_cal_commandline POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_LIST_DIR}/test_file_for_offline_mag.bin"
        "$<TARGET_FILE_DIR:offline_mag_cal_commandline>/test_file_for_offline_mag.bin"
)
