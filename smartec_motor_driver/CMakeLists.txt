cmake_minimum_required(VERSION 3.8)
project(smartec_motor_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(smartec_msgs REQUIRED)

include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
scripts/smartec_motor_driver.py
scripts/can_frames_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()