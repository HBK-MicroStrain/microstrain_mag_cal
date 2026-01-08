target_include_directories(${CALIBRATION_APP_BACKEND} PUBLIC ".")

target_sources(${CALIBRATION_APP_BACKEND}
    PRIVATE
        "backend.cpp"
        "backend.hpp"
)
target_sources(${CALIBRATION_APP_CLI}
    PRIVATE
        "cli.cpp"
        "cli.hpp"
        "main.cpp"
)