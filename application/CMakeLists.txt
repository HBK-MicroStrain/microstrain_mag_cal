cmake_minimum_required(VERSION 3.19)
project(offline_mag_cal
    VERSION 0.0.0
)

add_executable(offline_mag_cal_commandline
    "placeholder.cpp"
)

target_link_libraries(offline_mag_cal_commandline PRIVATE microstrain_mag_cal)