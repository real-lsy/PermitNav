cmake_minimum_required(VERSION 3.8)
project(go2w_cmd_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use local Unitree SDK2 installed under this workspace.
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/go2w_nav_ws/third_party/unitree_sdk2/install")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2w_cmd_bridge src/go2w_cmd_bridge.cpp)

ament_target_dependencies(go2w_cmd_bridge
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(go2w_cmd_bridge
  unitree_sdk2
)

install(TARGETS go2w_cmd_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
