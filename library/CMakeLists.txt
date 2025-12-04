# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

# NOTE: I had to do: `git config --global http.sslBackend schannel` to get the SSL certificates to
# work in windows. Might be worth looking into whether it's better to do this or update OpenSSL
# certificates manually.
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.0
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(Eigen)

# ---------------------------------------------------------------
# Library setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

project(microstrain_mag_cal VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress unused variable warnings for Python paths. This is to counteract the annoying
# behavior of automatically passing the Python path variables by some IDEs. We aren't
# using Python as part of the project, only side scripts, so the variables aren't needed.
if(DEFINED Python3_EXECUTABLE)
    # Mark as used
endif()
if(DEFINED Python_EXECUTABLE)
    # Mark as used
endif()

set(LIBRARY "microstrain_mag_cal")

add_library(${LIBRARY})

target_include_directories(${LIBRARY} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

set_target_properties(${LIBRARY} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
)

target_link_libraries(${LIBRARY} PUBLIC
    Eigen3::Eigen
)

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

target_compile_options(${LIBRARY} PRIVATE
    "${COMPILE_OPTIONS}"
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(microstrain_mag_cal)
add_subdirectory(test)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

# Install the executable
install(TARGETS ${LIBRARY}
    DESTINATION bin
    COMPONENT library
)

# ---------------------------------------------------------------
# Packaging
# ---------------------------------------------------------------

# Only package the required components
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};${PROJECT_NAME};library;/")
set(CPACK_PACKAGE_NAME ${LIBRARY})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "MicroStrain by HBK")
# TODO: Set reusable description for README and here
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library for offline magnetometer calibration.")

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
add_custom_target("package_${LIBRARY}"
    # Want to specify the config on multi-config generators, but not single-config ones.
    COMMAND ${CMAKE_CPACK_COMMAND} --config CPackConfig.cmake
    $<$<BOOL:$<CONFIG>>:-C> $<$<BOOL:$<CONFIG>>:$<CONFIG>>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Packaging ${LIBRARY}..."
)
