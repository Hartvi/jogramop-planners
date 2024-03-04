cmake_minimum_required(VERSION 3.2.0)

project(jogramop-planners)

# Set C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
# Find urdfdom and KDL parser packages
find_package(urdfdom REQUIRED)
# find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

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


file(GLOB_RECURSE SOURCES "src/*.cc")
# set(CMAKE_VERBOSE_MAKEFILE ON)


# Create the executable
add_executable(jogramop-planners ${SOURCES})

# Include and link urdfdom
target_include_directories(jogramop-planners PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(jogramop-planners ${urdfdom_LIBRARIES})

# Include and link kdl_parser and orocos_kdl
# target_include_directories(jogramop-planners PRIVATE ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_include_directories(jogramop-planners PRIVATE ${kdl_parser_INCLUDE_DIRS})
# target_link_libraries(jogramop-planners ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(jogramop-planners ${kdl_parser_LIBRARIES})

# target_link_libraries(jogramop-planners ${PYTHON_LIBRARIES})

# Link against PQP and other required libraries
target_link_libraries(jogramop-planners PQP flann lz4)

# Find and link the filesystem library if necessary
find_library(FILESYSTEM_LIBRARY NAMES stdc++fs)
if(FILESYSTEM_LIBRARY)
    target_link_libraries(jogramop-planners ${FILESYSTEM_LIBRARY})
endif()
