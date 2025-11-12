target_include_directories(microstrain_mag_cal PUBLIC
    "."
)

target_sources(microstrain_mag_cal PRIVATE
    "analysis.hpp"
    "analysis.cpp"
    "calibration.cpp"
    "calibration.hpp"
    "preprocessing.hpp"
    "preprocessing.cpp"
)