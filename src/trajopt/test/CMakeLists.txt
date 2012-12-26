include_directories(${GTEST_INCLUDE_DIRS})
add_definitions(-DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(collision-checker-unit collision-checker-unit.cpp)
target_link_libraries(collision-checker-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY})
add_test(collision-checker-unit ${CMAKE_BINARY_DIR}/bin/collision-checker-unit)

add_executable(planning-unit planning-unit.cpp)
target_link_libraries(planning-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY})
add_test(planning-unit ${CMAKE_BINARY_DIR}/bin/planning-unit)


include_directories(${JSON_INCLUDE_DIRS})
add_executable(json_experiment1 json_experiment1.cpp)
target_link_libraries(json_experiment1 json)

add_executable(json_experiment2 json_experiment2.cpp)
target_link_libraries(json_experiment2 json)

add_executable(json_experiment3 json_experiment3.cpp)
target_link_libraries(json_experiment3 trajopt gtest gtest_main)