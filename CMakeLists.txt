cmake_minimum_required(VERSION 3.8)
project(kinect_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(Threads REQUIRED)

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(camera_info_manager REQUIRED)

add_library(kinect_ros2_component SHARED src/kinect_ros2_component.cpp)
target_compile_definitions(kinect_ros2_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(kinect_ros2_component 
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  camera_info_manager
  rclcpp_components
  ament_index_cpp
  image_transport
  OpenCV)
target_link_libraries(kinect_ros2_component
  freenect)
rclcpp_components_register_nodes(kinect_ros2_component "kinect_ros2::KinectRosComponent")

add_executable(kinect_ros2_node
  src/kinect_ros2_node.cpp)
target_link_libraries(kinect_ros2_node
  kinect_ros2_component)
ament_target_dependencies(kinect_ros2_node
  "rclcpp")

install(TARGETS
  kinect_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  kinect_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY cfg launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
