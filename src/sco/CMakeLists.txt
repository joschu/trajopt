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
  
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64 bits
  	set(BPMPD_LIBRARY "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")
  else()
  	set(BPMPD_LIBRARY "${CMAKE_SOURCE_DIR}/3rdpartylib/bpmpd_linux32.a")
  endif()
  
  target_link_libraries(bpmpd_caller ${BPMPD_LIBRARY})
  list(APPEND SCO_SOURCE_FILES bpmpd_interface.cpp)  
  set_property(SOURCE bpmpd_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_CALLER="\\\"${CMAKE_BINARY_DIR}/bin/bpmpd_caller\\\"")


  set(BPMPD_WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE bpmpd_caller.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_WORKING_DIR="${BPMPD_WORKING_DIR}")
  file(COPY bpmpd.par DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


  set_property(SOURCE solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_BPMPD )
endif()

if (GUROBI_FOUND)
  set_property(SOURCE solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_GUROBI )
  list(APPEND SCO_SOURCE_FILES gurobi_interface.cpp)
endif(GUROBI_FOUND)

add_library(sco
  ${SCO_SOURCE_FILES}
)


set (SCO_LINK_LIBS utils)
if (GUROBI_FOUND)  
  list(APPEND SCO_LINK_LIBS ${GUROBI_LIBRARIES}) 
endif()
if (HAVE_BPMPD) 
  list(APPEND SCO_LINK_LIBS ${BPMPD_LIBRARY})
endif()

target_link_libraries(sco ${SCO_LINK_LIBS})


add_subdirectory(test)
