cmake_minimum_required(VERSION 3.0.2)
project(dofbot_tracker)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    yahboomcar_msgs
    sensor_msgs
    cv_bridge
)

## Catkin configuration
catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs yahboomcar_msgs sensor_msgs cv_bridge
)

## Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
catkin_install_python(PROGRAMS
    nodes/dofbot_arm_tracker.py
    nodes/dofbot_lib.py
    nodes/test_standalone_tracker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(FILES
    launch/arm_tracker.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dofbot_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
