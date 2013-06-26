
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

set(RBDL_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/ext/rbdl/src" "${CMAKE_BINARY_DIR}/ext/rbdl/src")

include_directories(
  SYSTEM
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
)

# add_executable(polygon_expt polygon_expt.cpp)

#add_executable(quaternion_optimization quaternion_optimization.cpp o3.cpp)
#target_link_libraries(quaternion_optimization trajopt)

# add_executable(rigid_body_simulation rigid_body_simulation.cpp)
# target_link_libraries(rigid_body_simulation trajopt)

add_executable(pushing pushing.cpp o3.cpp)
target_link_libraries(pushing trajopt humanoids)

add_executable(compl_test compl_test.cpp complementarity.cpp)
target_link_libraries(compl_test sco)

add_executable(dubins_car dubins_car.cpp)
target_link_libraries(dubins_car trajopt)

add_executable(needle_steering needle_steering.cpp o3.cpp)
target_link_libraries(needle_steering trajopt)
