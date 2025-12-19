target_include_directories(${BACKEND} PUBLIC ".")

target_sources(${BACKEND}
    PRIVATE
        "backend.cpp"
        "backend.hpp"
        "cli.cpp"
        "cli.hpp"
)
target_sources(${CLI}
    PRIVATE
        "main.cpp"
)