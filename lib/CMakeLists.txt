cmake_minimum_required(VERSION 3.19)

# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(json)

# ---------------------------------------------------------------
# Library Setup
# ---------------------------------------------------------------

# Suppress unused variable warnings for Python paths.
if(DEFINED Python3_EXECUTABLE)
    # Mark as used
endif()
if(DEFINED Python_EXECUTABLE)
    # Mark as used
endif()

add_library(${LIB_CALIBRATION})
add_library(${LIB_APPLY_CALIBRATION})

# Needed to avoid double "lib" on Linux, and follow platform convention on Windows.
set_target_properties(${LIB_CALIBRATION} PROPERTIES OUTPUT_NAME ${CALIBRATION_APP})
set_target_properties(${LIB_APPLY_CALIBRATION} PROPERTIES OUTPUT_NAME ${APPLY_CALIBRATION_APP})

target_include_directories(${LIB_CALIBRATION} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(${LIB_APPLY_CALIBRATION} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

set_target_properties(${LIB_CALIBRATION} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
)
set_target_properties(${LIB_APPLY_CALIBRATION} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
)

target_link_libraries(${LIB_CALIBRATION}
    PUBLIC
        Eigen3::Eigen
        magic_enum::magic_enum
        nlohmann_json::nlohmann_json
)
target_link_libraries(${LIB_APPLY_CALIBRATION}
    PUBLIC
        ${LIB_CALIBRATION}
)

target_compile_options(${LIB_CALIBRATION} PRIVATE ${COMPILE_OPTIONS})
target_compile_options(${LIB_APPLY_CALIBRATION} PRIVATE ${COMPILE_OPTIONS})

# ---------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------

add_subdirectory(${PROJECT_NAME})
add_subdirectory(${APPLY_CALIBRATION_APP})
add_subdirectory(test)

# ---------------------------------------------------------------
# Installation
# ---------------------------------------------------------------

install(TARGETS ${LIB_CALIBRATION}
    DESTINATION bin
    COMPONENT lib
)
install(TARGETS ${LIB_APPLY_CALIBRATION}
    DESTINATION bin
    COMPONENT lib
)
