cmake_minimum_required(VERSION 3.19)
project(microstrain_mag_cal
    VERSION 0.0.0
)

add_library(microstrain_mag_cal)

set_target_properties(microstrain_mag_cal PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    LINKER_LANGUAGE CXX
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

add_subdirectory(src)
add_subdirectory(test)
