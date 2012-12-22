find_package(json_spirit REQUIRED)

include_directories(
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${json_spirit_INCLUDE_DIR}
)
message("poo ${json_spirit_INCLUDE_DIR}")
add_library(trajopt SHARED 
	problem_description.cpp
	bullet_collision_checker.cpp 
	kinematic_constraints.cpp 
	robot_and_dof.cpp 
	utils.cpp
)

target_link_libraries(trajopt ${BULLET_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} sco util)

# set(CMAKE_SHARED_LINKER_FLAGS "-export-symbols-regex,'^foo_'")

add_library(trajoptplugin SHARED plugin.cpp)
set_target_properties(trajoptplugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(trajoptplugin ${OpenRAVE_LIBRARIES} trajopt)

add_subdirectory(py)
add_subdirectory(test)
