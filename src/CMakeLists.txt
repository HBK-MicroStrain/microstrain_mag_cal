target_include_directories(microstrain_mag_cal PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(microstrain_mag_cal PRIVATE
    "microstrain_mag_cal.cpp"
    "microstrain_mag_cal.hpp"
)