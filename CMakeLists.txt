cmake_minimum_required(VERSION 3.8)
project(kinect_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(Threads REQUIRED)

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(kinect_ros2 src/kinect_ros2_node.cpp)
ament_target_dependencies(kinect_ros2 
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV)
target_link_libraries(kinect_ros2
  freenect)

install(TARGETS
  kinect_ros2
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

ament_package()
