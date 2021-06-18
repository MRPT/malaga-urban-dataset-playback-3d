PROJECT(play_log_3d)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
      cmake_policy(SET CMP0005 NEW) 
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE( MRPT REQUIRED base obs maps topography gui)

ADD_DEFINITIONS(-DMY_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Declare the target (an executable)
ADD_EXECUTABLE(play_log_3d
	main.cpp
	)
TARGET_LINK_LIBRARIES(play_log_3d ${MRPT_LIBS})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native -Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


