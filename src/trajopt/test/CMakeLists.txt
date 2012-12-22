include_directories(${GTEST_INCLUDE_DIRS})
add_definitions(-DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(collision-checker-unit collision-checker-unit.cpp)
target_link_libraries(collision-checker-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY})

add_test(collision-checker-unit ${CMAKE_BINARY_DIR}/bin/collision-checker-unit)