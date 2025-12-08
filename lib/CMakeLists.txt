# ---------------------------------------------------------------
# Library Setup
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

# Suppress unused variable warnings for Python paths.
if(DEFINED Python3_EXECUTABLE)
    # Mark as used
endif()
if(DEFINED Python_EXECUTABLE)
    # Mark as used
endif()

set(LIB_CORE "mag_cal_core")

add_library(${LIB_CORE})

target_include_directories(${LIB_CORE} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

set_target_properties(${LIB_CORE} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
)

target_link_libraries(${LIB_CORE} PUBLIC Eigen3::Eigen)

target_compile_options(${LIB_CORE} PRIVATE ${COMPILE_OPTIONS})

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(${LIB_CORE})
add_subdirectory(test)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

# Install the executable
install(TARGETS ${LIB_CORE}
    DESTINATION bin
    COMPONENT lib
)
