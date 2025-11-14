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
project(microstrain_mag_cal
    VERSION 0.0.0
)

# Suppress unused variable warnings for Python paths. This is to counteract the annoying
# behavior of automatically passing the Python path variables by some IDEs. We aren't
# using Python as part of the project, only side scripts, so the variables aren't needed.
if(DEFINED Python3_EXECUTABLE)
    # Mark as used
endif()
if(DEFINED Python_EXECUTABLE)
    # Mark as used
endif()

add_library(microstrain_mag_cal)

target_include_directories(microstrain_mag_cal PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

set_target_properties(microstrain_mag_cal PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
)

target_link_libraries(microstrain_mag_cal PUBLIC
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

target_compile_options(microstrain_mag_cal PRIVATE
    "${COMPILE_OPTIONS}"
)

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(microstrain_mag_cal)
add_subdirectory(test)
