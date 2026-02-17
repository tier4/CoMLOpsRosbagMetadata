cmake_minimum_required(VERSION 3.8)
project(co_mlops_rosbag_metadata)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Entry point script so ros2 run and launch find co_mlops_metadata_publisher_node
set(WRAPPER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/co_mlops_metadata_publisher_node")
file(WRITE "${WRAPPER_SCRIPT}" "#!/usr/bin/env python3
import sys
try:
    from co_mlops_rosbag_metadata.co_mlops_metadata_publisher_node import main
    main()
except SystemExit as e:
    sys.exit(e.code if e.code is not None else 0)
")
file(CHMOD "${WRAPPER_SCRIPT}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${WRAPPER_SCRIPT}" DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/fixtures DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  add_launch_test(test/test_metadata_publisher_node.py)
  ament_add_pytest_test(test_invalid_yaml_exit_code test/test_invalid_yaml_exit_code.py)
endif()

ament_package()
