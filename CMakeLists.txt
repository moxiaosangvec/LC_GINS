cmake_minimum_required(VERSION 3.0.0)
project(lc_gins VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
include_directories(/usr/local/include/eigen3)

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)
add_library(gilib SHARED ${DIR_SRCS})

add_executable(lc_gins_node lc_gins_node.cpp)
target_link_libraries(lc_gins_node gilib)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

