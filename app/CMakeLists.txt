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

set(BACKEND "${PROJECT_NAME}_backend")      # Main application logic
set(CLI "${PROJECT_NAME}_cli")              # Command-line interface for app
set(APPLY_CLI "${PROJECT_NAME}_apply_cli")  # Command-line interface for applying calibration

add_library(${BACKEND} STATIC)
add_executable(${CLI})
add_executable(${APPLY_CLI})

string(REPLACE "_" "-" EXECUTABLE_NAME ${PROJECT_NAME})
set_target_properties(${CLI} PROPERTIES OUTPUT_NAME ${EXECUTABLE_NAME})
set_target_properties(${APPLY_CLI} PROPERTIES OUTPUT_NAME "${EXECUTABLE_NAME}-apply")

target_link_libraries(${BACKEND}
    Eigen3::Eigen
    "lib${PROJECT_NAME}"
    ${MIP_LIBRARIES}
)
target_link_libraries(${CLI}
    PRIVATE
        ${BACKEND}
        CLI11::CLI11
        mio::mio
)
target_link_libraries(${APPLY_CLI}
    PRIVATE
        ${BACKEND}
        CLI11::CLI11
        Eigen3::Eigen
        ${MIP_LIBRARIES}
        ${MICROSTRAIN_LIBRARIES}
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(${PROJECT_NAME})
add_subdirectory("${PROJECT_NAME}_apply")
add_subdirectory(test)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

install(TARGETS ${CLI}
    DESTINATION bin
    COMPONENT app
)

install(TARGETS ${APPLY_CLI}
    DESTINATION bin
    COMPONENT app
)
