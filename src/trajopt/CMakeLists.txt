include_directories(
  SYSTEM
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)

if (NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

add_library(trajopt
	traj_plotter.cpp
  trajectory_costs.cpp
	kinematic_terms.cpp 
	collision_terms.cpp
	json_marshal.cpp
	problem_description.cpp
	bullet_collision_checker.cpp 
	configuration_space.cpp 
	utils.cpp
	rave_utils.cpp
	collision_checker.cpp
	plot_callback.cpp
	bullet_unity.cpp
)
target_link_libraries(trajopt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)

add_subdirectory(test)

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(ctrajoptpy trajoptpy.cpp)
target_link_libraries(ctrajoptpy trajopt ${Boost_SYSTEM_LIBRARY} )


