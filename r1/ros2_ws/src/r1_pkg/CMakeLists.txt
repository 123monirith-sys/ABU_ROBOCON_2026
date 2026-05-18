cmake_minimum_required(VERSION 3.8)
project(r1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find dependencies
# =============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msg REQUIRED)
find_package(nlohmann_json REQUIRED)

# =============================
# Main executables
# =============================

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(talker2 src/test.cpp)
ament_target_dependencies(talker2 rclcpp std_msgs)

add_executable(listener2 src/test2.cpp)
ament_target_dependencies(listener2 rclcpp std_msgs)

add_executable(game_pad_node src/game_pad.cpp)
ament_target_dependencies(game_pad_node
  rclcpp
  std_msgs
  robot_msg
  nlohmann_json
)
add_executable(r1_menual_node src/r1_menual.cpp)
ament_target_dependencies(r1_menual_node
  rclcpp
  std_msgs
  robot_msg
 
)
add_executable(r1_cantx_node src/canr1_tx.cpp)
ament_target_dependencies(r1_cantx_node
  rclcpp
  std_msgs
  robot_msg
)
add_executable(r1_canrx_node src/canr1_rx.cpp)
ament_target_dependencies(r1_canrx_node
  rclcpp
  std_msgs
  robot_msg
)
include_directories(include)

add_executable(menual_node
  src/main.cpp
  src/menual_node.cpp
)

ament_target_dependencies(menual_node
  rclcpp
  robot_msg
)

# =============================
# Install
# =============================

install(TARGETS
  talker
  listener
  talker2
  listener2
  game_pad_node
  r1_menual_node
  r1_cantx_node
  r1_canrx_node
  menual_node
  DESTINATION lib/${PROJECT_NAME}
)

# =============================
# Testing
# =============================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()