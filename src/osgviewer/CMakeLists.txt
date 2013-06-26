
find_package( OpenThreads )
find_package( osg )
find_package( osgDB )
find_package( osgUtil )
find_package( osgViewer )
find_package(osgGA)       
       
set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil osgText)       
       
include_directories(
  SYSTEM  
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
)

if (APPLE) 
# see http://forum.openscenegraph.org/viewtopic.php?t=11097
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -fvisibility=hidden")
endif()

                    
add_library(osgviewer
  osgviewer.cpp 
  robot_ui.cpp
)
target_link_libraries(osgviewer ${OSG_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} utils)


#add_subdirectory(test)