cmake_minimum_required(VERSION 3.8)
project(image_latency_test)

add_compile_options(-g -O3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_publisher_node
  src/image_publisher_node.cpp
)
ament_target_dependencies(image_publisher_node
  rclcpp
  sensor_msgs
)

add_executable(image_subscriber_node
  src/image_subscriber_node.cpp
)
ament_target_dependencies(image_subscriber_node
  rclcpp
  sensor_msgs
)

add_executable(intra_process_image_latency
  src/intra_process_image_latency.cpp
)
ament_target_dependencies(intra_process_image_latency
  rclcpp
  sensor_msgs
)

install(TARGETS
  image_publisher_node
  image_subscriber_node
  intra_process_image_latency
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
