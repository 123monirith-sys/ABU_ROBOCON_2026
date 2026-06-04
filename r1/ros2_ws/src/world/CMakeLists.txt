cmake_minimum_required(VERSION 3.8)
project(world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(NODES
  swerve_controller_node
  heading_lock_node
  odom_node
  pos_controller_node
)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} rclcpp std_msgs geometry_msgs)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()

install(TARGETS ${NODES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
