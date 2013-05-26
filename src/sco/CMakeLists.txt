find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})



set(SCO_SOURCE_FILES
  solver_interface.cpp
	modeling.cpp
	expr_ops.cpp
	expr_vec_ops.cpp
	optimizers.cpp
	modeling_utils.cpp
	num_diff.cpp
)

if (NOT APPLE)
  set (HAVE_BPMPD TRUE)
endif()

if (HAVE_BPMPD)
  add_executable(bpmpd_caller bpmpd_caller.cpp)
  target_link_libraries(bpmpd_caller "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")
  list(APPEND SCO_SOURCE_FILES bpmpd_interface.cpp)  
  set_source_files_properties( bpmpd_interface.cpp PROPERTIES COMPILE_FLAGS -DBPMPD_CALLER="\\\"${CMAKE_BINARY_DIR}/bin/bpmpd_caller\\\"") 
  set_source_files_properties( solver_interface.cpp PROPERTIES COMPILE_FLAGS -DHAVE_BPMPD )
endif()

if (GUROBI_FOUND)
  set_source_files_properties( solver_interface.cpp PROPERTIES COMPILE_FLAGS -DHAVE_GUROBI )
  list(APPEND SCO_SOURCE_FILES gurobi_interface.cpp)
endif(GUROBI_FOUND)

add_library(sco
  ${SCO_SOURCE_FILES}
)



target_link_libraries(sco ${GUROBI_LIBRARIES} utils )


add_subdirectory(test)