cmake_minimum_required(VERSION 3.8)
project(camera)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories("/opt/ros/humble/include")

add_executable(camera src/camera_capture.cpp)
ament_target_dependencies(camera rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  camera 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
