
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

include_directories(
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)

#add_executable(cloud_convex_decomp_experiment cloud_convex_decomp_experiment.cpp)
#target_link_libraries(cloud_convex_decomp_experiment trajopt cloudproc osgviewer ${OpenRAVE_LIBRARIES})

# add_executable(polygon_expt polygon_expt.cpp)

add_executable(quaternion_optimization quaternion_optimization.cpp)
target_link_libraries(quaternion_optimization trajopt)