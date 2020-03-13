cmake_minimum_required(VERSION 2.8.3)
project(rosbot_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY maps
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(wallFollow samples/wallFollow.cpp samples/pid_core.cpp)
target_link_libraries(wallFollow ${catkin_LIBRARIES})


