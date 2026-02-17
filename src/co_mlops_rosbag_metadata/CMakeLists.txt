cmake_minimum_required(VERSION 3.8)
project(co_mlops_rosbag_metadata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(co_mlops_metadata_publisher_node src/co_mlops_rosbag_metadata_publisher_node.cpp)
target_include_directories(co_mlops_metadata_publisher_node PRIVATE include)
ament_target_dependencies(co_mlops_metadata_publisher_node rclcpp std_msgs)
target_link_libraries(co_mlops_metadata_publisher_node yaml-cpp)

install(TARGETS
  co_mlops_metadata_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY test/fixtures
  DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_metadata_publisher_node.py)
  add_launch_test(test/test_metadata_publisher_node_invalid_yaml.py)
endif()

ament_package()
