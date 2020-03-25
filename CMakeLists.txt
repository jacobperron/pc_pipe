cmake_minimum_required(VERSION 3.5)
project(pc_pipe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/66.pc_pipe.dsv.in)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS pc_pipe/image_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(etc)

ament_package()
