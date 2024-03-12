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
find_package(yaml-cpp REQUIRED)

# find_package(PythonLibs REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})


# Include directories
include_directories(include /usr/local/include /usr/include /usr/local/include/eigen3)

add_compile_options(-O2)     

#for profilling, slows down the code a lot!!!!    
# add_compile_options(-ggdb3 -pg )     
# add_link_options(-ggdb3 -pg )     


# 2. cmake .
# 3. make clean
# 4. make
# 5. spust program, treba pres ./run_jrbt.sh
# 6. vysledek je v gmon.out
# 7. gprof ./burs_of_free_space gmon.out > gmon.prof   #mozna budes muset zmenit cestu k binarce?
# 8. vysledek je v gmon.prof, coz je textak, ktery popisuje co kde jak dlouho trva

# Pro graficke zobrazeni pouzivam : https://github.com/jrfonseca/gprof2dot

# 9. Stahni gprof2dot a jdi do slozky, kam jsi ho stahnul, rozbalil
# 10.  python3 gprof2dot.py -n 5  -o burs.dot cesta/k/souboru/gmon.prof   #toto vytvori soubor burs.dot v tzv. 'dot' formatu
# 11.  dot -Tpng burs.dot > burs.png

# V kroku 10 volba "-n 5" znamena, ze se neukazuji volani, ktere celkove trvaly mene nez 5 procent casu. S toudle volbou je potreba si obcas pohrat..


# Add subdirectory for PQP
add_subdirectory(../PQP PQP_build)

# Gather source files
# file(GLOB SOURCES "src/*.cc")



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