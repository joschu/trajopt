cmake_minimum_required(VERSION 2.8)
project(trajoptrave)

include(cmake/main.cmake)

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(OpenRAVE 0.9 REQUIRED)

if (NOT CMAKE_BUILD_TYPE) 
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(BUILD_SHARED_LIBS true)
set(BULLET_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/bullet/src)
set(BULLET_LIBRARIES BulletCollision LinearMath)
set(GTEST_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/gtest/include)
set(JSON_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/json/include)

enable_testing()

option(BUILD_CLOUDPROC "build point cloud processing stuff" OFF)
option(BUILD_SANDBOX "build sandbox programs" OFF)

add_subdirectory(ext)
add_subdirectory(src)
