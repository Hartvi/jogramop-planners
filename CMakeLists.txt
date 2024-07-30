cmake_minimum_required(VERSION 3.2.0)

project(jogramop-planners)

## SET NINJA AS BUILD TOOL --- CAN BREAK STUFF => CHECK THIS
#set(CMAKE_MAKE_PROGRAM /usr/bin/ninja CACHE STRING "Ninja location")
if(NOT CMAKE_GENERATOR)
    set(CMAKE_GENERATOR "Unix Makefiles" CACHE STRING "Default generator")
endif()


# Set C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
# Find urdfdom and KDL parser packages
find_package(urdfdom REQUIRED)
# find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(PythonLibs REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})


# Include directories
include_directories(include /usr/local/include /usr/include /usr/local/include/eigen3)

add_compile_options(-O2)

#for profilling, slows down the code a lot!!!!    
#add_compile_options(-ggdb3 -pg )     
#add_link_options(-ggdb3 -pg )     


# Add subdirectory for PQP
add_subdirectory(../PQP PQP_build)

# Gather source files
# file(GLOB SOURCES "src/*.cc")


# add all .cc files as source code, they will then find the *.h files from the above include_directories
file(GLOB_RECURSE SOURCES "src/*.cc")

# set(CC_DIR
#     src/main.cc
# )

# set(SOURCES ${CC_DIR})


# Create the executable
add_executable(jogramop-planners ${SOURCES})

# Include and link urdfdom
target_include_directories(jogramop-planners PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(jogramop-planners PRIVATE ${urdfdom_LIBRARIES})

# Include and link kdl_parser and orocos_kdl
# target_include_directories(jogramop-planners PRIVATE ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_include_directories(jogramop-planners PRIVATE ${kdl_parser_INCLUDE_DIRS})
# target_link_libraries(jogramop-planners ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(jogramop-planners PRIVATE ${kdl_parser_LIBRARIES})

# target_link_libraries(jogramop-planners ${PYTHON_LIBRARIES})

# Link against PQP and other required libraries
target_link_libraries(jogramop-planners PRIVATE PQP flann lz4)


# Find and link the filesystem library if necessary
find_library(FILESYSTEM_LIBRARY NAMES stdc++fs)
if(FILESYSTEM_LIBRARY)
    target_link_libraries(jogramop-planners ${FILESYSTEM_LIBRARY})
endif()

# include(FetchContent)

# FetchContent_Declare(
#   yaml-cpp
#   GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
#   GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
# )
# FetchContent_GetProperties(yaml-cpp)

# if(NOT yaml-cpp_POPULATED)
#   message(STATUS "Fetching yaml-cpp...")
#   FetchContent_Populate(yaml-cpp)
#   add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
# endif()

# target_link_libraries(jogramop-planners PRIVATE yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library
target_include_directories(jogramop-planners PRIVATE ${yaml-cpp_INCLUDE_DIRS})
target_link_libraries(jogramop-planners PRIVATE yaml-cpp) # The library or executable that require yaml-cpp library
