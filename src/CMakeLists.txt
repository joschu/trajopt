include_directories(
  ${CMAKE_SOURCE_DIR}/ext
  ${CMAKE_SOURCE_DIR}/src
)


add_definitions(-DEIGEN_DEFAULT_TO_ROW_MAJOR)

find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message(FATAL_ERROR "Gurobi not found. Set GUROBI_HOME variabe")
else()
include_directories(${GUROBI_INCLUDE_DIRS})
endif()

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_subdirectory(sco)
add_subdirectory(trajopt)
add_subdirectory(utils)
if(BUILD_CLOUDPROC) 
	add_subdirectory(cloudproc)
endif()
if(BUILD_SANDBOX)
	add_subdirectory(sandbox)
endif()
add_subdirectory(osgviewer)
if(BUILD_HUMANOIDS)
  add_subdirectory(humanoids)
endif()

if(NOT APPLE)
endif()
