cmake_minimum_required(VERSION 3.13)
project(dvrpalpha)
set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES src/main.cpp src/problem.cpp src/ant_colony.cpp src/ant.cpp src/tour_atom.cpp src/local_search.cpp)

add_executable(dvrpalpha ${SOURCE_FILES})