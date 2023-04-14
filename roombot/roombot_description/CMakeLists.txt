cmake_minimum_require(VERSION 3.5)
project(roombot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
	PATTER ".svn" EXCLUDE
)

install(DIRECTORY urdf/
	DESTINATION share/${PROJECT_NAME}/urdf
	PATTERN ".svn" EXCLUDE
)

ament_package()
