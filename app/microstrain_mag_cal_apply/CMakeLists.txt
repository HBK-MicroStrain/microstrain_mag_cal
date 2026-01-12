target_include_directories(${APPLY_CALIBRATION_APP_BACKEND} PUBLIC ".")

target_sources(${APPLY_CALIBRATION_APP_BACKEND}
    PRIVATE
        "backend.cpp"
        "backend.hpp"
)
target_sources(${APPLY_CALIBRATION_APP_CLI}
    PRIVATE
        "cli.cpp"
        "cli.hpp"
        "main.cpp"
)
