target_include_directories(${APP} PUBLIC ".")

target_sources(${CLI}
    PRIVATE
        "cli.cpp"
)
target_sources(${APP}
    PRIVATE
        "app.cpp"
        "app.hpp"
)