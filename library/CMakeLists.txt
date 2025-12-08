# ---------------------------------------------------------------
# Library setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

project(microstrain_mag_cal VERSION ${REPO_NUMERIC_VERSION})
set(PROJECT_VERSION ${REPO_FULL_VERSION})

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
