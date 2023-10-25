cmake_minimum_required(VERSION 3.2.0)

project(burs_of_free_space)

# set debugging
set(CMAKE_BUILD_TYPE Debug)

# Set C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include /usr/local/include /usr/include /usr/local/include/eigen3)  # ../PQP/include/

# Add subdirectory for PQP
add_subdirectory(../PQP PQP_build)

# Gather source files
file(GLOB SOURCES "src/*.cc")

# Create the executable
add_executable(burs_of_free_space ${SOURCES})

# Link against PQP and other required libraries
target_link_libraries(burs_of_free_space PQP flann lz4)

# Find and link the filesystem library if necessary
find_library(FILESYSTEM_LIBRARY NAMES stdc++fs)
if(FILESYSTEM_LIBRARY)
    target_link_libraries(burs_of_free_space ${FILESYSTEM_LIBRARY})
endif()
