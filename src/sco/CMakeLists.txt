find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(sco SHARED
  solver_interface.cpp
  bpmpd_interface.cpp
	gurobi_interface.cpp
	modeling.cpp
	expr_ops.cpp
	expr_vec_ops.cpp
	optimizers.cpp
	modeling_utils.cpp
	num_diff.cpp
)
target_link_libraries(sco ${GUROBI_LIBRARIES} utils boost_filesystem boost_serialization boost_system )

add_executable(bpmpd_caller bpmpd_caller.cpp )
target_link_libraries(bpmpd_caller "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a" boost_serialization boost_system boost_filesystem )

add_subdirectory(test)