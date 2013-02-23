find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(sco SHARED
	solver_interface.cpp
	gurobi_interface.cpp
	modeling.cpp
	expr_ops.cpp
	expr_vec_ops.cpp
	optimizers.cpp
	modeling_utils.cpp
	num_diff.cpp
)
target_link_libraries(sco ${GUROBI_LIBRARIES} utils)

add_subdirectory(test)