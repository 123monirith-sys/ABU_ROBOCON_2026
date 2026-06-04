cmake_minimum_required(VERSION 3.8)
project(kinematic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- ekf_node ---
add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp std_msgs geometry_msgs)
target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# --- ik_node ---
add_executable(ik_node src/ik_node.cpp)
ament_target_dependencies(ik_node rclcpp std_msgs geometry_msgs)
target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ik_node nlohmann_json::nlohmann_json)

# --- swerve_can_node ---
add_executable(swerve_can_node src/swerve_can_node.cpp)
ament_target_dependencies(swerve_can_node
  rclcpp std_msgs geometry_msgs)
target_include_directories(swerve_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(swerve_can_node nlohmann_json::nlohmann_json pthread)

install(TARGETS ekf_node ik_node swerve_can_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
