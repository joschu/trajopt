
       
set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil)       
       
include_directories(
  SYSTEM  
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -fvisibility=hidden")


                    
add_library(sensorsim
  sim_kinect.cpp
)
target_link_libraries(sensorsim ${OSG_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} utils osgviewer)


add_executable(sim_kinect_expt sim_kinect_expt.cpp)
target_link_libraries(sim_kinect_expt sensorsim osgviewer)

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(sensorsimpy sensorsimpy.cpp)
target_link_libraries(sensorsimpy osgviewer ${Boost_SYSTEM_LIBRARY} sensorsim)


