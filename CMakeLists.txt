cmake_minimum_required(VERSION 3.19)
project(parent)
enable_testing()

include(cmake/git_version.cmake)
get_project_version_from_git_tag("v" REPO_FULL_VERSION REPO_NUMERIC_VERSION)

add_subdirectory(library)
add_subdirectory(application)