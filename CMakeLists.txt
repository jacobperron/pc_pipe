cmake_minimum_required(VERSION 3.5)
project(pc_pipe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/66.pc_pipe.dsv.in)

add_executable(mock_point_cloud_pub
  src/mock_point_cloud_pub.cpp
)
ament_target_dependencies(mock_point_cloud_pub
  "image_transport"
  "message_filters"
  "rclcpp"
  "sensor_msgs"
)
install(TARGETS mock_point_cloud_pub
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS pc_pipe/image_filter.py pc_pipe/image_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(etc)

ament_package()
