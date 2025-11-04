target_include_directories(offline_mag_cal_core PUBLIC ".")

target_sources(offline_mag_cal_cli
    PRIVATE
        "cli.cpp"
)
target_sources(offline_mag_cal_core
    PRIVATE
        "core.cpp"
        "core.hpp"
)