include_directories(
  ${CMAKE_SOURCE_DIR}/ext
  ${CMAKE_SOURCE_DIR}/src
)

# todo: FindGurobi.cmake
set(GUROBI_HOME $ENV{GUROBI_HOME} CACHE STRING "Gurobi directory (e.g. /path/to/gurobi501/linux64)")
if ("${GUROBI_HOME}" STREQUAL "")
        message(STATUS "GUROBI_HOME not set. Gurobi will be disabled")
else()
        message(STATUS "Gurobi will be enabled")
        include_directories("${GUROBI_HOME}/include")
        set(GUROBI_LIBRARIES "${GUROBI_HOME}/lib/libgurobi50.so")
        add_definitions("-DHAVE_GUROBI")
endif()

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_subdirectory(ipi)
add_subdirectory(trajopt)
add_subdirectory(utils)
add_subdirectory(osgviewer)
add_subdirectory(sandbox)

if(NOT APPLE)
	add_subdirectory(cloudproc)
endif()
