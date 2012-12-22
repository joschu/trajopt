boost_python_module(trajoptpy trajoptpy.cpp)
target_link_libraries(trajoptpy trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_PYTHON_LIBRARY})