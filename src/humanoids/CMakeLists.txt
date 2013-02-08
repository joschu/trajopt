
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)


#add_executable(robot_dynamics_experiment robot_dynamics_experiment.cpp)
#target_link_libraries(robot_dynamics_experiment sco trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenRAVE_BOTH_LIBRARIES})

add_library(humanoids humanoids.cpp hull2d.cpp)
target_link_libraries(humanoids sco trajopt ${OpenRAVE_BOTH_LIBRARIES})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

add_executable(zmp_test zmp_test.cpp)
target_link_libraries(zmp_test  sco trajopt humanoids ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenRAVE_BOTH_LIBRARIES})

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(humanoidspy humanoidspy.cpp)
target_link_libraries(humanoidspy trajopt humanoids ${Boost_SYSTEM_LIBRARY} )

