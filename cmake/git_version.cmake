# Extracts the version for a project from the latest git tag.
#
# Each project can be tagged with a prefix, such as "lib/v1.2.3". This function extracts the unique
# version for that project, even if it exists in the same repo as another project.
#
# Arguments:
#     PROJECT_PREFIX - The project prefix in the git tag ("lib" in "lib/v1.2.3").
#
# Returns:
#     VERSION_OUT - The extracted version string (ex: "1.2.3").
#
function(get_project_version_from_git_tag PROJECT_PREFIX VERSION_OUT)
    execute_process(
        COMMAND git describe --tags --match "${PROJECT_PREFIX}/v*" --always --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(GIT_TAG)
        # Strip project and 'v' (e.g., "lib/v1.2.3" -> "1.2.3")
        string(REGEX REPLACE "^${PROJECT_PREFIX}/v" "" VERSION_STRING ${GIT_TAG})
    else()
        message(WARNING "No git tag found matching '${PROJECT_PREFIX}/v*', using 0.0.0 as a fallback.")
        set(${OUTPUT_VAR} "0.0.0" PARENT_SCOPE)
    endif()
endfunction()
