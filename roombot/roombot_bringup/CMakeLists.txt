cmake_minimum_required(VERSION 3.5)
project(roombot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
	PATTERN ".svn>" EXCLUDE
)

ament_package()
