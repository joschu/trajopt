find_package(Numpy)
find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})


boost_python_module(ctrajoptpy trajoptpy.cpp)
target_link_libraries(ctrajoptpy trajopt ${Boost_SYSTEM_LIBRARY} )

#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/trajoptpy DESTINATION ${CMAKE_BINARY_DIR})
#install(FILES ${CMAKE_BINARY_DIR}/lib/ctrajoptpy.so DESTINATION ${CMAKE_BINARY_DIR}/trajoptpy)

