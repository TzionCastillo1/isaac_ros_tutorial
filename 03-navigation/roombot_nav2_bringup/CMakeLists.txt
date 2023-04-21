cmake_minimum_required(VERSION 3.5)
project(roombot_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY params/
	DESTINATION share/${PROJECT_NAME}/params
	PATTERN ".svn" EXCLUDE
)

ament_package()
