

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(cloudproc cloudproc.cpp)
target_link_libraries (cloudproc ${PCL_LIBRARIES})

add_executable(surface_recon_test surface_recon_test.cpp)
target_link_libraries(surface_recon_test cloudproc ${PCL_LIBRARIES})
