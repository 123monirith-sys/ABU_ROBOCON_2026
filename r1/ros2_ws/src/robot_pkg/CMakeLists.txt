cmake_minimum_required(VERSION 3.8)
project(robot_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(robot_msg REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# include headers
include_directories(include)

# add executables
add_executable(canrx_node src/canrx_node.cpp)
add_executable(cantx_node src/cantx_node.cpp)
add_executable(main_node src/main_node.cpp)
add_executable(game_pad src/game_pad.cpp)

# add libraries
add_library(ekf_lib src/ekf_lib.cpp)
add_library(trajectory_loader src/trajectory_loader.cpp)
add_library(pose_control_lib src/pose_control_lib.cpp)
# add_library(stm32_status src/stm32_status.cpp)

# ament_target_dependencies(stm32_status
#   std_msgs
# )

# link dependencies
ament_target_dependencies(canrx_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
  robot_msg
  rosidl_default_runtime
)

  ament_target_dependencies(cantx_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
  robot_msg
  rosidl_default_runtime
  )

  ament_target_dependencies(main_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
  robot_msg
  rosidl_default_runtime
  )

  ament_target_dependencies(game_pad
  rclcpp
  std_msgs
  robot_msg
)

target_link_libraries(game_pad
  jsoncpp
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_file
  DESTINATION share/${PROJECT_NAME}
)

# library
target_link_libraries(main_node
  ekf_lib
  trajectory_loader
  pose_control_lib
  # stm32_status
)

  install(TARGETS
  canrx_node
  cantx_node
  main_node
  ekf_lib
  trajectory_loader
  pose_control_lib
  game_pad
  # stm32_status
  DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
  include/robot_pkg/ekf_lib.hpp
  include/robot_pkg/pose_control_lib.hpp
  include/robot_pkg/trajectory_loader.hpp
  # include/robot_pkg/stm32_status.hpp
  DESTINATION include/${PROJECT_NAME}
  )
  install(FILES
  launch/main.launch.py
  DESTINATION share/${PROJECT_NAME}
  )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
