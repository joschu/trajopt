include_directories(${GTEST_INCLUDE_DIRS})
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

set(test_srcs
  unit.cpp
  small-problems-unit.cpp
  solver-interface-unit.cpp
)

add_executable(ipi-sco-unit
  ${test_srcs}
)    

target_link_libraries(ipi-sco-unit
  sco
  gtest gtest_main
)

add_test(ipi-sco-unit
  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ipi-sco-unit
)
