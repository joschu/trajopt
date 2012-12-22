cmake_minimum_required(VERSION 2.8)
project(trajoptrave)

include(cmake/main.cmake)

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system python thread REQUIRED)
find_package(OpenRAVE REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(IPI_SOURCE_DIR $ENV{IPI_SOURCE_DIR} CACHE STRING "")
set(IPI_BUILD_DIR $ENV{IPI_BUILD_DIR} CACHE STRING "")

set(BULLET_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/bullet/src)
set(BULLET_LIBRARIES BulletCollision LinearMath)
set(GTEST_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/gtest/include)

enable_testing()


add_subdirectory(ext)
add_subdirectory(src)
