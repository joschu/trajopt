include_directories(${GTEST_INCLUDE_DIRS})
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

add_executable(collision-checker-unit collision-checker-unit.cpp)
target_link_libraries(collision-checker-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY})
add_test(collision-checker-unit ${CMAKE_BINARY_DIR}/bin/collision-checker-unit)

add_executable(planning-unit planning-unit.cpp)
target_link_libraries(planning-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} osgviewer)
add_test(planning-unit ${CMAKE_BINARY_DIR}/bin/planning-unit)


add_executable(render-collision-test render_collision_test.cpp)
target_link_libraries(render-collision-test trajopt ${Boost_SYSTEM_LIBRARY} osgviewer)

add_executable(cast-cost-unit cast-cost-unit.cpp)
target_link_libraries(cast-cost-unit trajopt gtest gtest_main ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} osgviewer)
add_test(cast-cost-unit ${CMAKE_BINARY_DIR}/bin/cast-cost-unit)