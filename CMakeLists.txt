cmake_minimum_required(VERSION 3.0.0)
project(lc_gins VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen include dirs: ${EIGEN3_INCLUDE_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/impl/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/impl/src DIR_SRCS)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/interface/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/interface/lib)

add_library(gilib SHARED ${DIR_SRCS})

# add_executable(gins_review ${PROJECT_SOURCE_DIR}/impl/app/gins_review.cpp)
# target_link_libraries(gins_review gilib)

