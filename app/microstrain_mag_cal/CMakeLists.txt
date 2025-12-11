target_include_directories(${BACKEND} PUBLIC ".")

target_sources(${BACKEND}
    PRIVATE
        "backend.cpp"
        "backend.hpp"
)
target_sources(${CLI}
    PRIVATE
        "cli.cpp"
)