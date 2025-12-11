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

set(BACKEND "${PROJECT_NAME}_backend")  # Main application logic
set(CLI "${PROJECT_NAME}_cli")          # Command-line interface for app

add_library(${BACKEND} STATIC)
add_executable(${CLI})

string(REPLACE "_" "-" EXECUTABLE_NAME ${PROJECT_NAME})
set_target_properties(${CLI} PROPERTIES OUTPUT_NAME ${EXECUTABLE_NAME})

target_link_libraries(${BACKEND}
    Eigen3::Eigen
    "lib${PROJECT_NAME}"
    ${MIP_LIBRARY}
)
target_link_libraries(${CLI}
    PRIVATE
        CLI11::CLI11
        mio::mio
        ${BACKEND}
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(${PROJECT_NAME})
add_subdirectory(test)
add_subdirectory(scripts)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

# Install the executable
install(TARGETS ${CLI}
    DESTINATION bin
    COMPONENT app
)
