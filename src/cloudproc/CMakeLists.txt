

find_package(PCL 1.7 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(SYSTEM ${HACD_INCLUDE_DIRS})


if (APPLE)
	include_directories("/usr/local/Cellar/vtk/5.10.0/include/vtk-5.10")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -Wno-deprecated")


# add_custom_command(
#   COMMAND "./gen_instantiations.py" 
#   OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/autogen_instantiations.cpp"
#   WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" 
#   DEPENDS gen_instantiations.py cloudproc.hpp
# )
# add_custom_target(instantiations ALL DEPENDS autogen_instantiations.cpp)

add_library(cloudproc 
  cloudproc.cpp  
  sphere_sampling.cpp
  mesh_simplification
  vtkQuadricDecimation2.cxx
  cloudgrabber.cpp
  hacd_unity.cpp
  hacd_interface.cpp
)
# add_dependencies(cloudproc instantiations)

target_link_libraries (cloudproc ${PCL_LIBRARIES})

boost_python_module(cloudprocpy cloudprocpy.cpp)
target_link_libraries(cloudprocpy cloudproc ${Boost_SYSTEM_LIBRARY} )

#add_executable(simplify_test simplify_test.cpp)
#target_link_libraries(simplify_test ${PCL_LIBRARIES} cloudproc ${PCL_LIBRARIES})
