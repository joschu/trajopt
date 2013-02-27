cmake_minimum_required(VERSION 2.8)
project(trajopt)

include(cmake/main.cmake)

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(OpenRAVE 0.9 REQUIRED)

# http://cmake.3232098.n2.nabble.com/Default-value-for-CMAKE-BUILD-TYPE-td7550756.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(BUILD_SHARED_LIBS true)
set(BULLET_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/bullet/src)
set(BULLET_LIBRARIES BulletCollision LinearMath)
set(GTEST_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/gtest/include)
set(JSON_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/json/include)
set(HACD_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/HACD_Lib/inc ${CMAKE_SOURCE_DIR}/ext/HACD_Lib/src)

enable_testing()

option(BUILD_CLOUDPROC "build point cloud processing stuff" OFF)
option(BUILD_SANDBOX "build sandbox programs" OFF)
option(BUILD_HUMANOIDS "build humanoids stuff" OFF)

add_subdirectory(ext)
add_subdirectory(src)

include("${CMAKE_SOURCE_DIR}/cmake/add_python_unit_tests.cmake")
