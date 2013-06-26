#add_executable(viewer_experiment viewer_experiment.cpp)
#target_link_libraries(viewer_experiment ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

#add_executable(osg_expt osg_expt.cpp)
#target_link_libraries(osg_expt ${OSG_LIBRARIES})

#add_executable(robotui_expt robotui_expt.cpp)
#target_link_libraries(robotui_expt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

# include_directories(${EIGEN_INCLUDE_DIRS})
# add_executable(sim_kinect_expt sim_kinect_expt.cpp ../sim_kinect.cpp)
# target_link_libraries(sim_kinect_expt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} osgviewer)

add_executable(test_osgviewer_text test_osgviewer_text.cpp)
target_link_libraries(test_osgviewer_text osgviewer)