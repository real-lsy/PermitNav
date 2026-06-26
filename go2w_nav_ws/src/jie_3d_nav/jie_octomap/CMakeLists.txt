cmake_minimum_required(VERSION 3.8)
project(jie_octomap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jie_map_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_library(TINYXML2_LIB tinyxml2 REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(octomap_test src/octomap_test.cpp)
ament_target_dependencies(octomap_test
  rclcpp
  octomap_msgs
  visualization_msgs
)
target_link_libraries(octomap_test octomap)

add_executable(octomap_to_cloud_node src/octomap_to_cloud_node.cpp)
ament_target_dependencies(octomap_to_cloud_node
  rclcpp
  octomap_msgs
  sensor_msgs
)
target_link_libraries(octomap_to_cloud_node octomap)

add_executable(octomap_to_occupied_markers_node src/octomap_to_occupied_markers_node.cpp)
ament_target_dependencies(octomap_to_occupied_markers_node
  rclcpp
  octomap_msgs
  visualization_msgs
)
target_link_libraries(octomap_to_occupied_markers_node octomap)

add_executable(occupancy_grid_to_octomap_node src/occupancy_grid_to_octomap_node.cpp)
ament_target_dependencies(occupancy_grid_to_octomap_node
  rclcpp
  nav_msgs
  octomap_msgs
)
target_link_libraries(occupancy_grid_to_octomap_node octomap)

add_executable(octomap_open3d_viewer_node src/octomap_open3d_viewer_node.cpp)
ament_target_dependencies(octomap_open3d_viewer_node
  rclcpp
  nav_msgs
  octomap_msgs
  visualization_msgs
)
target_link_libraries(octomap_open3d_viewer_node
  octomap
  Open3D::Open3D
)

add_executable(rviz_click_selector_node src/rviz_click_selector_node.cpp)
ament_target_dependencies(rviz_click_selector_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(world_to_octomap_node src/world_to_octomap_node.cpp)
ament_target_dependencies(world_to_octomap_node
  rclcpp
  octomap_msgs
  std_msgs
  visualization_msgs
)
target_include_directories(world_to_octomap_node PRIVATE /usr/include/eigen3)
target_link_libraries(world_to_octomap_node
  octomap
  ${TINYXML2_LIB}
)

add_executable(pcd_to_octomap_node src/pcd_to_octomap_node.cpp)
ament_target_dependencies(pcd_to_octomap_node
  rclcpp
  octomap_msgs
  std_msgs
)
target_link_libraries(pcd_to_octomap_node
  octomap
  Open3D::Open3D
)

install(TARGETS
  octomap_test
  octomap_to_cloud_node
  octomap_to_occupied_markers_node
  occupancy_grid_to_octomap_node
  octomap_open3d_viewer_node
  rviz_click_selector_node
  world_to_octomap_node
  pcd_to_octomap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/open3d_octomap_viewer.py
  scripts/pcd_file_publisher.py
  scripts/no_cache_http_server.py
  scripts/save_bin_map_gui.py
  scripts/world_file_picker_gui.py
  scripts/world_selector_gui.py
  scripts/web_click_selector.py
  scripts/map_package_manager
  scripts/map_save_gui
  scripts/map_viewer_gui
  scripts/ros_map_import_gui
  scripts/pcd_map_import_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config web worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
