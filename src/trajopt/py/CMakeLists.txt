find_package(Numpy)
include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(trajoptpy trajoptpy.cpp)

FIND_PACKAGE(PythonLibs REQUIRED)

target_link_libraries(trajoptpy trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_PYTHON_LIBRARY})