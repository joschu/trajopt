add_executable(viewer_experiment viewer_experiment.cpp)
target_link_libraries(viewer_experiment ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

add_executable(osg_expt osg_expt.cpp)
target_link_libraries(osg_expt ${OSG_LIBRARIES})