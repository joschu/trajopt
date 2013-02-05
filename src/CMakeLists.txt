include_directories(
  ${CMAKE_SOURCE_DIR}/ext
  ${CMAKE_SOURCE_DIR}/src
)


find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message(FATAL_ERROR "Gurobi not found. Set GUROBI_HOME variabe")
else()
include_directories(${GUROBI_INCLUDE_DIRS})
endif()

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_subdirectory(ipi)
add_subdirectory(trajopt)
add_subdirectory(utils)
if(BUILD_CLOUDPROC) 
	add_subdirectory(cloudproc)
endif()
if(BUILD_SANDBOX)
	add_subdirectory(sandbox)
endif()
add_subdirectory(osgviewer)
add_subdirectory(humanoids)

if(NOT APPLE)
endif()
