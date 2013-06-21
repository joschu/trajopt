
set(RBDL_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/ext/rbdl/src" "${CMAKE_BINARY_DIR}/ext/rbdl/src")


include_directories(
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
#  ${RBDL_INCLUDE_DIRS}
)

if (APPLE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -fvisibility=hidden")
endif()

#add_executable(robot_dynamics_experiment robot_dynamics_experiment.cpp)
#target_link_libraries(robot_dynamics_experiment sco trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenRAVE_BOTH_LIBRARIES})

add_library(humanoids humanoids.cpp 
	hull2d.cpp 
	#rave_to_rbdl.cpp
	)
target_link_libraries(humanoids sco trajopt ${OpenRAVE_BOTH_LIBRARIES})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DBIGDATA_DIR="${CMAKE_SOURCE_DIR}/bigdata")

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(humanoidspy humanoidspy.cpp)
target_link_libraries(humanoidspy trajopt humanoids ${Boost_SYSTEM_LIBRARY} )


include_directories(${GTEST_INCLUDE_DIRS})
#add_executable(rbdl-unit rbdl-unit.cpp)
#target_link_libraries(rbdl-unit trajopt humanoids gtest gtest_main)
