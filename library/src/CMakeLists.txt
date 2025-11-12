target_include_directories(microstrain_mag_cal PUBLIC
    "."
)

target_sources(microstrain_mag_cal PRIVATE
    "microstrain_mag_cal.cpp"
    "microstrain_mag_cal.hpp"
    "preprocessing.hpp"
    "preprocessing.cpp"
)