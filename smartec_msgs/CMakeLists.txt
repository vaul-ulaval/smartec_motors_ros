cmake_minimum_required(VERSION 3.8)
project(smartec_msgs)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/SmartecStatus.msg"
	DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()