cmake_minimum_required(VERSION 3.19)

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
FetchContent_MakeAvailable(json)
FetchContent_MakeAvailable(mio)
FetchContent_MakeAvailable(mip_sdk)

# ---------------------------------------------------------------
# Application setup
# ---------------------------------------------------------------

set(CALIBRATION_APP_BACKEND "${CALIBRATION_APP}_backend")
set(CALIBRATION_APP_CLI "${CALIBRATION_APP}_cli")

set(APPLY_CALIBRATION_APP_BACKEND "${APPLY_CALIBRATION_APP}_backend")
set(APPLY_CALIBRATION_APP_CLI "${APPLY_CALIBRATION_APP}_cli")

add_library(${CALIBRATION_APP_BACKEND} STATIC)
add_library(${APPLY_CALIBRATION_APP_BACKEND} STATIC)
add_executable(${CALIBRATION_APP_CLI})
add_executable(${APPLY_CALIBRATION_APP_CLI})

string(REPLACE "_" "-" EXECUTABLE_NAME ${CALIBRATION_APP})
set_target_properties(${CALIBRATION_APP_CLI} PROPERTIES OUTPUT_NAME ${EXECUTABLE_NAME})
set_target_properties(${APPLY_CALIBRATION_APP_CLI} PROPERTIES OUTPUT_NAME "${EXECUTABLE_NAME}-apply")

target_link_libraries(${CALIBRATION_APP_BACKEND}
    Eigen3::Eigen
    ${MIP_LIBRARIES}
    ${LIB_CALIBRATION}
    mio::mio
)
target_link_libraries(${APPLY_CALIBRATION_APP_BACKEND}
    ${LIB_APPLY_CALIBRATION}
    ${LIB_CALIBRATION}
    ${MIP_LIBRARIES}
    ${MICROSTRAIN_LIBRARIES}
)
target_link_libraries(${CALIBRATION_APP_CLI}
    PRIVATE
        CLI11::CLI11
        ${CALIBRATION_APP_BACKEND}
        magic_enum::magic_enum
)
target_link_libraries(${APPLY_CALIBRATION_APP_CLI}
    PRIVATE
        ${APPLY_CALIBRATION_APP_BACKEND}
        CLI11::CLI11
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(${CALIBRATION_APP})
add_subdirectory(${APPLY_CALIBRATION_APP})
add_subdirectory(test)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

install(TARGETS ${CALIBRATION_APP_CLI}
    DESTINATION bin
    COMPONENT app
)

install(TARGETS ${APPLY_CALIBRATION_APP_CLI}
    DESTINATION bin
    COMPONENT app
)
