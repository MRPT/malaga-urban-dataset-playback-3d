project(play_log_3d)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package(MRPT REQUIRED core obs maps topography gui)

# Declare the target (an executable)
add_executable(${PROJECT_NAME}
    main.cpp
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${MRPT_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    MY_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

# Set optimized building:
if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
        -O3 -mtune=native -Wall
        )
endif()
