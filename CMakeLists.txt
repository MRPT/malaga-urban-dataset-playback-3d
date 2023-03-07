project(malaga_urban_dataset_examples)

cmake_minimum_required(VERSION 3.0)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
      cmake_policy(SET CMP0005 NEW)
endif()

add_subdirectory(play_log_3d)
add_subdirectory(project-lidar-into-image)