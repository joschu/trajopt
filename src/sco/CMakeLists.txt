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
target_link_libraries(sco ${GUROBI_LIBRARIES} utils )

add_executable(bpmpd_caller bpmpd_caller.cpp)
SET_SOURCE_FILES_PROPERTIES( bpmpd_interface.cpp PROPERTIES COMPILE_FLAGS -DBPMPD_CALLER="\\\"${CMAKE_BINARY_DIR}/bin/bpmpd_caller\\\"" )
target_link_libraries(bpmpd_caller "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")

add_subdirectory(test)