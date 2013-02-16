

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (APPLE)
	include_directories("/usr/local/Cellar/vtk/5.10.0/include/vtk-5.10")
endif()

add_library(cloudproc cloudproc.cpp  convexdecomp.cpp sphere_sampling.cpp
# cloudgrabber.cpp
convexdecomp2.cpp
)
target_link_libraries (cloudproc ${PCL_LIBRARIES})

#add_executable(surface_recon_test surface_recon_test.cpp)
#target_link_libraries(surface_recon_test cloudproc ${PCL_LIBRARIES})

#add_executable(convex_decomp_test convex_decomp_test.cpp)
#target_link_libraries(convex_decomp_test cloudproc ${PCL_LIBRARIES} utils)

#add_executable(opm_test opm_test.cpp)
#target_link_libraries(opm_test cloudproc ${PCL_LIBRARIES} utils)


#add_executable(test_print test_print.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
boost_python_module(cloudprocpy cloudprocpy.cpp)
target_link_libraries(cloudprocpy cloudproc ${Boost_SYSTEM_LIBRARY} )
