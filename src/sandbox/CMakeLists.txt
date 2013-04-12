
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

set(RBDL_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/ext/rbdl/src" "${CMAKE_BINARY_DIR}/ext/rbdl/src")

include_directories(
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
)

# add_executable(polygon_expt polygon_expt.cpp)

add_executable(quaternion_optimization quaternion_optimization.cpp)
target_link_libraries(quaternion_optimization trajopt)

# add_executable(rigid_body_simulation rigid_body_simulation.cpp)
# target_link_libraries(rigid_body_simulation trajopt)

add_executable(pushing pushing.cpp)
target_link_libraries(pushing trajopt humanoids)
