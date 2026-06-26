# jie_3d_nav

[English](./README.en.md)

一套基于 ROS 2 Humble 的 3D 导航系统，通过 Web 界面交互。本系统已在智元科技 D1 机器狗以及留形科技 Odin 1 空间定位模组上测试通过。

<p align="center">
  <img src="./media/1.png" alt="概略图" width="800">
</p>

本目录包含三个 ROS 2 包：

- `jie_map_msgs`：地图包保存、加载、导出等自定义服务接口。
- `jie_octomap`：OctoMap 管理包，负责多种地图格式导入、地图包保存/加载、OctoMap 可视化和编辑。
- `octo_planner`：基于 OctoMap 的 3D 路径规划、路径跟踪控制和 Web 测试/导航 launch。

## 功能概览

- 将 PCD 点云地图导入为 OctoMap。
- 将 ROS 2D 栅格地图导入为 3D OctoMap。
- 将 Gazebo `.world` / `.sdf` 场景转换为 OctoMap。
- 保存、加载 OctoMap 地图包。
- 使用 Qt/VTK GUI 查看和编辑 OctoMap 栅格。
- 使用 Web 页面查看 OctoMap、选择起点/终点并进行路径规划。
- 提供面向安装了留形科技 Odin 1 的 智元 D1 机器狗的导航入口和独立网页测试入口。

## 介绍视频

- Bilibili：[【开源】基于ROS2的3D导航系统](https://www.bilibili.com/video/BV1jgR9BmELw)
- YouTube：[【开源】基于ROS2的3D导航系统](https://www.youtube.com/watch?v=CepO90mzIeI)

## 目录结构

```text
jie_3d_nav/
├── jie_map_msgs/        # 自定义 srv 接口
├── jie_octomap/         # OctoMap 导入、管理、编辑、Web/GUI 工具
├── octo_planner/        # 3D 路径规划、控制器、导航 launch
├── jie_octomap/worlds/  # 示例 Gazebo world
└── install_deps_humble.sh
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- OctoMap / octomap_msgs
- OpenCV
- Open3D C++ 开发库
- PyQt5、VTK、NumPy、Pillow、PyYAML
- 可选：`rosbridge_server`，用于 Web 页面通过 websocket 访问 ROS

### ROS 2 Foxy 复现说明

本项目主要面向 Ubuntu 22.04 / ROS 2 Humble。Ubuntu 20.04 / ROS 2 Foxy 可用于验证核心链路（Gazebo world 或 PCD 地图导入为 OctoMap，再进行 3D 路径规划），但需要额外注意：

- 安装 Foxy 的 Python 点云工具包：`sudo apt-get install ros-foxy-sensor-msgs-py`。
- Ubuntu 20.04 默认提供 `python3-vtk7`，其 Qt 入口是 `vtk.qt.QVTKRenderWindowInteractor`；较新的 VTK 使用 `vtkmodules.qt.QVTKRenderWindowInteractor`。
- 如果 Open3D C++ 安装在系统路径之外，编译时需要通过 `Open3D_DIR` 或 `CMAKE_PREFIX_PATH` 指向 `Open3DConfig.cmake`。
- 如果运行 `pcd_to_octomap_node` 时出现 `libtbb.so.12: cannot open shared object file`，需要把 Open3D 安装目录下的 `lib/` 加入运行库路径，例如：

```bash
export LD_LIBRARY_PATH=/path/to/open3d_install/lib:${LD_LIBRARY_PATH}
```

基础编译不需要以下两个包：

- `d1_bringup`
- `d1_description`

注意：完整智元科技 D1 机器狗导航入口 `octo_planner/launch/nav.launch.py` 仍然会在运行时使用 `d1_bringup` 和 `d1_description`，因为它会启动 `d1_core` 并读取智元科技 D1 机器狗的 URDF。

## 安装依赖

可以使用仓库内脚本安装常用依赖：

```bash
cd ~/ros2_ws/src/jie_3d_nav
bash install_deps_humble.sh
```

如果 CMake 找不到 Open3D，需要额外安装 Open3D C++ 开发库，并确保 `Open3DConfig.cmake` 能被 CMake 找到，例如通过 `Open3D_DIR` 或 `CMAKE_PREFIX_PATH` 指定。

## 编译

从 ROS 2 工作区根目录编译。不要在源码包目录 `src/jie_3d_nav` 内直接执行 `colcon build`，否则会在源码目录下生成多余的 `build/`、`install/`、`log/`：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jie_map_msgs jie_octomap octo_planner
source install/setup.bash
```

如果 Open3D C++ 不在默认 CMake 搜索路径中：

```bash
colcon build --packages-select jie_map_msgs jie_octomap octo_planner \
  --cmake-args -DOpen3D_DIR=/path/to/open3d_install/lib/cmake/Open3D
```

如果源码目录移动过，旧 CMake 缓存可能还指向旧路径，可以清理缓存后重编：

```bash
colcon build --packages-select jie_map_msgs jie_octomap octo_planner --cmake-clean-cache
```

如果误在源码包目录内编译过，可以删除源码目录下被 `.gitignore` 忽略的临时产物：

```bash
rm -rf build install log
```

## 快速体验

运行如下指令加载例子地图：

```bash
ros2 launch jie_octomap import_gazebo_world.launch.py world_name:=2_storey.world
```

在弹出的窗口中，加载gazebo的world文件，设置地图的截取边长，点击加载按钮即可将world文件转换成OctoMap，并显示在窗口中。

<p align="center">
  <img src="./media/2.png" alt="加载地图" width="600">
</p>

先点击“起始点”按钮，用鼠标在地图上设置起始点位置。再点击“目标点”按钮，用鼠标在地图上设置目标点位置，即可规划出可行的路径。

<p align="center">
  <img src="./media/3.png" alt="规划路径" width="600">
</p>

## 地图导入

### 导入 PCD 点云地图

```bash
ros2 launch jie_octomap import_pcd_map.launch.py
```

该 launch 会启动：

- `pcd_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `pcd_map_import_gui`
- `octo_planner/jie_path_node`

`pcd_map_import_gui` 支持读取 PCD 后在左侧预览点云，并在转换前做常用清理：

- `推荐转换参数`：根据当前点云的点间距、点数和地图范围，自动填入 OctoMap 分辨率、每体素最少点数和最小连通体素数。
- `预处理降采样(m)`：读取 PCD 时对 GUI 工作点云进行体素降采样。修改该值后需要重新选择/读取 PCD 才会应用到当前点云。
- `启用选区方块`：显示可移动选区方块，`W/S`、`A/D`、`Q/E` 分别沿 X/Y/Z 移动。
- `抹除框内点云`：删除选区方块内的点。
- `仅保留框内点云`：保留选区方块内的点，移除外部点，适合从大范围点云中裁剪出待转换区域。

稀疏或大范围 PCD 建议先点击 `推荐转换参数`，再转换为 OctoMap。转换后重点观察终端日志中的 `kept_voxels`、`occupied_voxels`：

- `kept_voxels` 只有几十或几百，通常表示分辨率过细或 `每体素最少点数` 过高。
- `kept_voxels` 特别大，通常表示分辨率过细或裁剪范围过大，Web/Qt 显示和规划会变慢。
- 稀疏点云的起步配置通常使用 `每体素最少点数=1`、`最小连通体素数=1`。

转换成功时，终端应看到类似日志：

```text
Loaded PCD file: ... source_points=... kept_voxels=... occupied_voxels=...
Preprocess mask rebuilt...
Preblocked costmap rebuilt...
```

保存地图包时，GUI 默认根目录为当前用户的 `~/maps`。如果手动选择保存目录，确保该目录属于当前用户并可写，不要使用只适用于作者机器人环境的 `/home/robot/maps`。

如果 GUI 右侧没有显示转换后的 OctoMap，或保存地图包时提示 `not ready` / `octomap not ready`，优先检查终端中 `pcd_to_octomap_node` 是否已经退出。常见原因是 Open3D 依赖的运行库没有被动态链接器找到，例如：

```text
error while loading shared libraries: libtbb.so.12: cannot open shared object file
```

这时先确认依赖是否可见：

```bash
ldd install/jie_octomap/lib/jie_octomap/pcd_to_octomap_node | grep -E "not found|tbb"
```

如果 `libtbb.so.12` 未找到，把 Open3D 安装目录下的 `lib/` 加入 `LD_LIBRARY_PATH` 后重新启动 launch：

```bash
export LD_LIBRARY_PATH=/path/to/open3d_install/lib:${LD_LIBRARY_PATH}
ros2 launch jie_octomap import_pcd_map.launch.py
```

也可以在命令行直接覆盖转换节点的默认参数：

```bash
ros2 launch jie_octomap import_pcd_map.launch.py \
  resolution:=0.5 \
  voxel_downsample_m:=0.0 \
  min_points_per_voxel:=1 \
  min_cluster_voxels:=1
```

### 导入 ROS 2D 栅格地图

```bash
ros2 launch jie_octomap import_ros_map.launch.py
```

该 launch 会启动：

- `occupancy_grid_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `ros_map_import_gui`
- `octo_planner/jie_path_node`

### 导入 Gazebo World / SDF

加载包内示例 world 时，推荐使用 `world_name`：

```bash
ros2 launch jie_octomap import_gazebo_world.launch.py world_name:=field.world
```

加载外部 world 文件时，继续使用绝对路径：

```bash
ros2 launch jie_octomap import_gazebo_world.launch.py world_file:=/absolute/path/to/map.world
```

如果同时传入 `world_file` 和 `world_name`，优先使用 `world_file`。

`jie_octomap/worlds/` 目录内提供了两个示例 world 文件，并会随 `jie_octomap` 包安装到 `share/jie_octomap/worlds/`：

- `2_storey.world`：双层建筑/楼层示例。
- `field.world`：场地示例。

加载双层建筑示例：

```bash
ros2 launch jie_octomap import_gazebo_world.launch.py world_name:=2_storey.world
```

加载场地示例：

```bash
ros2 launch jie_octomap import_gazebo_world.launch.py world_name:=field.world
```

该 launch 会启动：

- `world_to_octomap_node`
- `world_selector_gui.py`
- `map_package_manager`
- `octo_planner/jie_path_node`

## 地图管理与编辑

OctoMap 管理和编辑主入口：

```bash
ros2 launch jie_octomap map_manager.launch.py
```

该 launch 会启动：

- `map_package_manager`
- `octomap_to_occupied_markers_node`
- `map_viewer_gui`
- 可选 `octo_planner/jie_path_node`

`map_viewer_gui` 支持：

- 打开地图包
- 刷新地图
- 保存地图
- 查看占据、禁行、可通行、风险代价图层
- 编辑栅格：`occupied`、`preblocked`、`traversable`、`clear`
- 选择起点、终点、导航目标

## Web 可视化

### 加载地图并启动 Web 页面

```bash
ros2 launch jie_octomap web_octomap.launch.py map_package:=~/maps/map
```

常用参数：

- `map_package`：已保存的地图包目录。
- `http_port`：静态 Web 服务端口，默认 `8080`。
- `launch_rosbridge`：是否启动 `rosbridge_websocket`。
- `launch_map_gui`：是否同时启动 Qt 保存/加载窗口。

如果 `8080` 已被占用，会出现 `OSError: [Errno 98] Address already in use`。这不是实机或机器人 IP 问题，换一个端口即可：

```bash
ros2 launch jie_octomap web_octomap.launch.py \
  map_package:=~/maps/map \
  http_port:=8081
```

如果需要网页与 ROS 通信，启动 `rosbridge_websocket`：

```bash
ros2 launch jie_octomap web_octomap.launch.py \
  map_package:=~/maps/map \
  http_port:=8081 \
  launch_rosbridge:=true
```

如果系统未安装 rosbridge：

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```

浏览器访问：

```text
http://localhost:8081
```

如果从另一台设备访问，使用运行该 launch 的电脑 IP，例如 `http://<电脑IP>:8081`。只有 Web 服务运行在机器人上时才使用机器人 IP。

### Web 功能测试

```bash
ros2 launch octo_planner web_test.launch.py
```

`web_test.launch.py` 用于测试网页访问、地图显示、Web 起终点选择、路径规划和基础控制链路。该 launch 已去除对 `d1_bringup` 和 `d1_description` 的依赖，会使用一个最小 `base_link` URDF 启动 `robot_state_publisher`。

启动前同样需要根据实际环境配置：

```text
octo_planner/config/nav_params.yaml
```

至少需要部署好：

- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

## 智元科技 D1 机器狗完整导航

完整机器人导航入口：

```bash
ros2 launch octo_planner nav.launch.py
```

该 launch 面向智元科技 D1 机器狗实际导航，并结合留形科技 Odin 1 空间定位模组相关驱动流程，会启动或使用：

- `d1_bringup/d1_core`
- `d1_description/urdf/d1.urdf`
- `odin_ros_driver`
- `octo_planner/jie_path_node`
- `octo_planner/d1_controller`
- `jie_octomap/map_package_manager`
- Web viewer 和 `rosbridge_websocket`

运行前需要根据实际环境修改：

```text
octo_planner/config/nav_params.yaml
```

重点字段：

- `relocalization_bin_file`
- `map_package_dir`
- `relocalization_pcd_file`
- `show_rviz`
- `show_map_gui`
- `publish_d1_odom`
- `use_static_odom_to_base`

同时需要确认留形科技 Odin 1 空间定位模组驱动配置：

```text
odin_ros_driver/config/control_command.yaml
```

将其中的 `custom_map_mode` 设置为 `2`，即 `Relocalization mode`。

`octo_planner/config/nav_params.yaml` 中至少需要部署好：

- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

如果需要使用 RViz 观察定位效果，还需要部署：

- `relocalization_pcd_file`：用于 RViz 显示的 `.pcd` 点云地图文件。

## 其他 Launch

```bash
ros2 launch jie_octomap octomap_test.launch.py
ros2 launch jie_octomap octomap_open3d.launch.py
ros2 launch jie_octomap odin1_slam.launch.py
ros2 launch jie_octomap odin1_loc.launch.py
```

其中 `odin1_slam.launch.py` 和 `odin1_loc.launch.py` 面向留形科技 Odin 1 空间定位模组流程，运行时需要 `odin_ros_driver`，并可选使用 `odin_costmap` 配置。

## 入门教材推荐

《机器人操作系统（ROS2）入门与实践》

<p align="center">
  <img src="./media/book_1.jpg" alt="机器人操作系统 ROS2 入门与实践" width="400">
</p>

淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 关注公众号

欢迎关注公众号，后续会继续带来更多有意思的机器人、ROS 2 和具身智能相关开源项目。

<p align="center">
  <img src="./media/AJQR.jpg" alt="公众号二维码" width="360">
</p>

