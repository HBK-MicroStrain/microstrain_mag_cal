add_executable(apply_calibration "apply_calibration.cpp")

target_link_libraries(apply_calibration
    ${MICROSTRAIN_LIBRARIES}
    ${MIP_LIBRARIES}
)
