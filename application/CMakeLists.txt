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

# ---------------------------------------------------------------
# Packaging
# ---------------------------------------------------------------

# Only package the required components
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};${PROJECT_NAME};application;/")
set(CPACK_PACKAGE_NAME "offline-mag-cal")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "MicroStrain by HBK")
# TODO: Set reusable description for README and here
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Commandline application for offline magnetometer calibration.")

if(WIN32)
    set(CPACK_GENERATOR "ZIP")
elseif(UNIX)
    set(CPACK_GENERATOR "TGZ")
endif()

# Force CPack to output its config in the current binary directory
set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackConfig.cmake")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackSourceConfig.cmake")

include(CPack)

# ---------------------------------------------------------------
# CI/CD
# ---------------------------------------------------------------

# Convenience target to build and package the application (commandline).
add_custom_target("package_${CLI}"
    # Want to specify the config on multi-config generators, but not single-config ones.
    COMMAND ${CMAKE_CPACK_COMMAND} --config CPackConfig.cmake 
            $<$<BOOL:$<CONFIG>>:-C> $<$<BOOL:$<CONFIG>>:$<CONFIG>>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Packaging ${CLI}..."
)
