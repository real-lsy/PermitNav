# PermitNav for go2w

本项目基于 **Unitree Go2-W / Go2W**、**Livox MID360**、**ROS2 Humble**、**FAST-LIO**、**FAST_LIO_LOCALIZATION2**、**jie_3d_nav** 和 **Unitree SDK2**，缝合适配速成了一套室内（似乎并不能太室内）3d全局自主导航。PermitNav，Permit——指速成出demo效果就能被放去实习了。项目仅为实现基础3d导航效果，主业loco没咋搞过nav，如遇bug，建议重开。

视频：
【【开源】PermitNav，缝合速成的一种全局3d导航】 https://www.bilibili.com/video/BV12Y7j6iEHi/?share_source=copy_web&vd_source=b5c22f93b0d2fd36b1d6203208bd650b

NUC 端负责：

```text
MID360 数据采集
  ↓
FAST-LIO 建图 / 实时里程计
  ↓
FAST_LIO_LOCALIZATION2 全局定位
  ↓
localization bridge 发布 map -> base_link
  ↓
jie_3d_nav 网页端规划与 controller
  ↓
go2w_cmd_bridge 转发 /cmd_vel 到 Go2-W
```

默认工作区：

```bash
~/go2w_nav_ws
```

## 致谢与开源项目引用

本项目并不是一个从零开始实现的导航框架，而是一个基于现有开源项目快速搭建的 ROS2 实机导航集成项目。

在此感谢以下开源项目的作者和维护者：

| 开源项目 | 在本项目中的作用 |
|---|---|
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | Livox 雷达底层 SDK，用于接收 MID360 等 Livox 雷达的数据。 |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Livox 雷达的 ROS/ROS2 驱动。本项目中用于发布 MID360 的点云数据和 IMU 数据。 |
| [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | FAST-LIO 的 ROS2 版本。本项目中用于激光惯性里程计、实时建图，并发布 `/Odometry` 和配准后的点云。 |
| [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) | 基于 FAST-LIO 地图的定位框架。本项目中基于该项目进行适配，用于在预先构建的 FAST-LIO 点云地图中进行重定位。 |
| [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) | 基于 ROS2 Humble 的三维导航系统，包含 Web 交互、OctoMap 地图管理、路径规划与控制器输出。本项目中用于网页端目标点下发、三维地图管理和路径规划。 |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | Unitree 官方 SDK2。本项目中由 `go2w_cmd_bridge` 调用，用于将 `/cmd_vel` 转换为 Go2-W 可执行的速度控制命令。 |
| [OctoMap](https://octomap.github.io/) | 三维占据栅格地图框架。本项目中用于三维环境表示和导航规划。 |

本仓库主要完成以下工作：

```text
1. Go2-W、MID360、NUC 的硬件部署与网络配置；
2. Livox、FAST-LIO、FAST_LIO_LOCALIZATION2、jie_3d_nav 和 Unitree SDK2 的系统集成；
3. FAST-LIO 点云地图的保存、处理和导入；
4. 定位结果到导航所需 TF 的桥接；
5. `/cmd_vel` 到 Go2-W SDK 控制接口的转发；
6. 一套可复现的实机启动脚本和运行流程。
```

本项目的核心定位是短周期工程集成，而不是从零提出新的导航算法。项目价值在于将多个现有 ROS2 开源项目连接成一套可复现、可实机运行的 Go2-W + MID360 自主导航链路。

相关算法、驱动和核心软件能力归属于各自上游开源项目。使用、修改、再发布或商业使用前，请阅读并遵守对应项目的开源许可证。

---

## 1. 硬件与网络

### 1.1 已验证硬件

| 设备   | 型号                   |
| ---- | -------------------- |
| 机器人  | Unitree Go2-W / Go2W |
| 激光雷达 | Livox MID360         |
| 主控   | N150 NUC / Ubuntu 22.04   |
| ROS  | ROS2 Humble          |

---

### 1.2 网络连接

推荐使用交换机：

```text
NUC enp2s0  →  交换机
MID360      →  交换机
Go2-W       →  交换机
```

当前使用的 IP：

| 设备              | IP                  |
| --------------- | ------------------- |
| NUC - Go2-W 网段  | `192.168.123.99/24` |
| NUC - MID360 网段 | `192.168.1.50/24`   |
| Go2-W           | `192.168.123.161`   |
| MID360          | `192.168.1.165`     |

---

### 1.3 配置 NUC 双 IP

本项目使用 `enp2s0` 同时连接 Go2-W 和 MID360 两个网段。

推荐新建一个专用连接：

```bash
在"设置-网络-有线"选项中点击右上角加号,添加名称为"go2w-mid360"的新配置
sudo nmcli connection modify go2w-mid360 ipv4.addresses "192.168.123.99/24,192.168.1.50/24"
sudo nmcli connection modify go2w-mid360 ipv4.method manual
sudo nmcli connection modify go2w-mid360 ipv4.gateway ""
sudo nmcli connection modify go2w-mid360 ipv4.dns ""
sudo nmcli connection modify go2w-mid360 ipv4.never-default yes
sudo nmcli connection modify go2w-mid360 connection.autoconnect yes
sudo nmcli connection up go2w-mid360
```

检查：

```bash
ip -br addr show enp2s0
ip route get 192.168.123.161
ip route get 192.168.1.165
ping -c 3 192.168.123.161
ping -c 3 192.168.1.165
```

理想情况：

```text
enp2s0 上同时存在：
192.168.123.99/24
192.168.1.50/24

Go2-W 路由源地址：
192.168.123.99

MID360 路由源地址：
192.168.1.50
```

注意：

```text
1. Go2-W 网段 IP 192.168.123.99/24 建议排在前面；
2. WiFi 不要使用 192.168.123.x 网段；
3. 如果 Go2-W 能 ping 通但 SDK 控制失败，优先检查 enp2s0 IP 顺序和 WiFi 网段冲突。
```

---

## 2. 安装依赖

### 2.1 基础依赖

```bash
sudo apt update

sudo apt install -y \
  git cmake make gcc g++ build-essential \
  python3-pip python3-setuptools python3-colcon-common-extensions \
  python3-rosdep python3-vcstool python3-argcomplete \
  wget curl unzip tmux gdb net-tools iproute2 pkg-config
```

如果遇到 apt 锁：

```bash
sudo systemctl stop unattended-upgrades || true
sudo systemctl stop apt-daily.service || true
sudo systemctl stop apt-daily-upgrade.service || true
sudo dpkg --configure -a
sudo apt update
```

---

### 2.2 ROS2 依赖

确保已经安装 ROS2 Humble，然后执行：

```bash
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-eigen \
  ros-humble-eigen3-cmake-module \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-message-filters \
  ros-humble-octomap \
  ros-humble-octomap-msgs \
  ros-humble-octomap-ros \
  ros-humble-octomap-server \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
  
sudo apt install -y ros-humble-tf-transformations python3-transforms3d

python3 -m pip install --user ros2_numpy
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果提示已经初始化过，直接执行：

```bash
rosdep update
```

---

### 2.3 C++ 库依赖

```bash
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  libopencv-dev \
  libboost-all-dev \
  libyaml-cpp-dev \
  libgoogle-glog-dev \
  libgflags-dev
```

---

### 2.4 Python 依赖

FAST_LIO_LOCALIZATION2 和部分地图工具需要 Python 依赖，尤其是 Open3D。

```bash
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-yaml
```

安装 Open3D：

```bash
python3 -m pip install --user open3d
```
若出现numpy版本报错:
```bash
python3 -m pip uninstall -y numpy scipy scikit-learn sklearn open3d

python3 -m pip install --user --force-reinstall \
  "numpy==1.23.5" \
  "scipy==1.10.1" \
  "scikit-learn==1.3.2" \
  "open3d==0.19.0"
```
不要让 numpy 升级到 2.x，否则 Ubuntu 22.04 的 scipy / sklearn / open3d 组合容易出现 ABI 冲突。

检查：

```bash
python3 - <<'PY'
import open3d as o3d
import numpy as np
import scipy
print("open3d:", o3d.__version__)
print("numpy:", np.__version__)
print("scipy:", scipy.__version__)
PY
```

如果出现：

```text
externally-managed-environment
```

则执行：

```bash
python3 -m pip install --user open3d --break-system-packages
```

如果安装后仍然找不到 Open3D：

```bash
echo 'export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

再次检查：

```bash
python3 -c "import open3d; print(open3d.__version__)"
```

这一步必须通过。

安装jie_3d_nav依赖
```bash
cd ~/go2w_nav_ws/src/jie_3d_nav
bash install_deps_humble.sh
```

---

## 3. 获取代码

推荐工作区结构：

```text
go2w_nav_ws
├── src
│   ├── livox_ros_driver2
│   ├── FAST_LIO_ROS2
│   ├── FAST_LIO_LOCALIZATION2
│   ├── jie_3d_nav
│   ├── go2w_cmd_bridge
│   └── go2w_sensor_tools
├── third_party
│   ├── Livox-SDK2
│   └── unitree_sdk2
├── scripts
└── maps
```

进入工作区：

```bash
cd ~/go2w_nav_ws
```

---

## 4. 编译工程
安装Livox-SDK2
```bash
cd ~/go2w_nav_ws/third_party/Livox-SDK2

rm -rf build
mkdir build
cd build

cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

安装Unitree SDK2
```bash
cd ~/go2w_nav_ws/third_party/unitree_sdk2

rm -rf build
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=../install

make -j$(nproc)
make install

echo 'export LD_LIBRARY_PATH=$HOME/go2w_nav_ws/third_party/unitree_sdk2/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

本项目中 `livox_ros_driver2` 使用官方方式编译。

实测 `./build.sh humble` 会触发当前工作区的 colcon 编译，并同时编译工作区内其他包。因此本项目推荐将它作为主编译方式。

执行：

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2

chmod +x build.sh

source /opt/ros/humble/setup.bash

./build.sh humble
```

编译完成后：

```bash
cd ~/go2w_nav_ws
source install/setup.bash
```

11建议写入环境变量：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $HOME/go2w_nav_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$HOME/go2w_nav_ws/third_party/unitree_sdk2/install/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc

source ~/.bashrc
```11

检查编译结果：

```bash
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 pkg list | grep fast_lio
ros2 pkg list | grep localization
ros2 pkg executables go2w_cmd_bridge
```

正常输出应包含：

```text
livox_ros_driver2/msg/CustomMsg 可以显示
fast_lio
fast_lio_localization
go2w_cmd_bridge go2w_cmd_bridge
```

如果 `./build.sh humble` 最后显示：

```text
Summary: xx packages finished
```

且没有：

```text
failed
aborted
```

说明编译成功。

中间出现 `stderr output` 不一定是错误，很多只是 CMake warning。

---

## 5. MID360 配置检查

当前 MID360 IP：

```text
192.168.1.165
```

NUC host IP：

```text
192.168.1.50
```

检查 Livox 配置：

```bash
grep -R "192.168.1" -n ~/go2w_nav_ws/src/livox_ros_driver2/config
```

配置中应对应：

```text
host_ip / cmd_data_ip / point_data_ip / imu_data_ip: 192.168.1.50
lidar_ip: 192.168.1.165
```

先确认网络：

```bash
ip route get 192.168.1.165
ping -c 3 192.168.1.165
```

---

## 6. 建图流程

第一次使用时，需要先使用 FAST-LIO 扫地图。

---

### 6.1 使用 PCD 保存版本的 FAST-LIO 启动脚本

当前仓库中可保存 PCD 的脚本为：

```bash
~/go2w_nav_ws/scripts/start_fastlio.sh.pcd_true
```

建图前复制为正式启动脚本：start_fastlio.sh

```bash
chmod +x ~/go2w_nav_ws/scripts/start_fastlio.sh
```

打开 FAST-LIO 的 PCD 保存：

```bash
sed -i 's/pcd_save_en:[[:space:]]*false/pcd_save_en: true/g' \
~/go2w_nav_ws/src/FAST_LIO_ROS2/config/mid360_nuc_mapping.yaml
```

如果 install 目录已经存在，也同步修改：

```bash
if [ -f ~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml ]; then
  sed -i 's/pcd_save_en:[[:space:]]*false/pcd_save_en: true/g' \
  ~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml
fi
```

关闭FAST-LIO 的 PCD 保存：
使用原版start_fastlio.sh
```bash
sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
~/go2w_nav_ws/src/FAST_LIO_ROS2/config/mid360_nuc_mapping.yaml

sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml
```


---

### 6.2 启动 Livox 和 FAST-LIO 扫图

终端 1：

```bash
chmod +x ~/go2w_nav_ws/scripts/start_livox.sh
~/go2w_nav_ws/scripts/start_livox.sh
```

终端 2：

```bash
chmod +x ~/go2w_nav_ws/scripts/start_fastlio.sh
~/go2w_nav_ws/scripts/start_fastlio.sh
```

检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic list | grep cloud
```

扫图建议：

```text
1. 尽量不要有人在地图中走动；
2. 机器人或雷达移动要平稳；
3. 走廊、门口、转角、楼梯附近多扫几次；
4. 扫图结束后正常关闭 FAST-LIO，等待 PCD 保存完成。
```

保存的地图：~/go2w_nav_ws/maps下fastlio_yyyymmdd_hhmmss

查看地图
```bash
sudo apt install pcl-tools
pcl_viewer ~/go2w_nav_ws/maps/fastlio_yyyymmdd_hhmmss/scans.pcd
```

将 `scans.pcd` 拷贝到 PC 端处理。

PC 端负责：

```text
1. 自动找平；
2. 降噪；
3. 使用 jie_3d_nav 转换 OctoMap；
4. 导出处理后的 localization PCD 和 jie_3d_nav 地图包。
```

具体处理方式见 PC 端 README。

---

## 7. 从 PC 导入处理后的地图

地图处理流程维护在另一个独立项目中：

```text
go2w_nav_pc_ws（coming soon）
```

PC 端项目负责处理 NUC 上由 FAST-LIO 扫描生成的原始点云地图。建图完成后，需要将 NUC 端生成的原始 `scans.pcd` 拷贝到 PC 工作区，并在 `go2w_nav_pc_ws` 中完成地图处理。

PC 端主要完成以下工作：

```text
1. 对原始 FAST-LIO 点云地图进行自动找平；
2. 去除漂浮点、离群点和明显噪声；
3. 生成 FAST_LIO_LOCALIZATION2 使用的定位 PCD；
4. 将处理后的 PCD 导入 jie_3d_nav；
5. 转换生成基于 OctoMap 的导航地图；
6. 导出最终的 localization PCD 和 jie_3d_nav 地图文件夹，并拷贝回 NUC。
```

PC 端处理完成后，需要将以下内容导回 NUC：

```text
1. big_localization_raw.pcd
   → 用于 FAST_LIO_LOCALIZATION2 定位

2. jie_3d_nav 导出的地图文件夹
   → 用于网页端规划和 controller 控制
```

具体的 PC 端地图处理步骤，请参考 `go2w_nav_pc_ws` 项目中的 README。

---

### 7.1 导入 localization 使用的 PCD

PC 端输出的处理后大地图：

```text
big_localization_raw.pcd
```

放到 NUC：

```bash
mkdir -p ~/go2w_nav_ws/maps/localization
```

目标路径：

```bash
~/go2w_nav_ws/maps/localization/big_localization_raw.pcd
```

在 PC 端执行：

```bash
scp ~/go2w_nav_pc_ws/maps/final/big_localization_raw.pcd \
  name@<IP>:~/go2w_nav_ws/maps/localization/
```

---

### 7.2 导入 jie_3d_nav 地图包

PC 端通过 jie_3d_nav 导出的 OctoMap 地图文件夹放到：

```bash
~/go2w_nav_ws/maps/jie_maps
```

在 NUC 上创建：

```bash
mkdir -p ~/go2w_nav_ws/maps/jie_maps
```

在 PC 端执行：

```bash
scp -r <PC端jie_3d_nav导出的地图文件夹> \
  name@<IP>:~/go2w_nav_ws/maps/jie_maps/
```

检查：

```bash
ls -lh ~/go2w_nav_ws/maps/localization/
ls -lh ~/go2w_nav_ws/maps/jie_maps/
```

---

### 7.3 检查地图路径

检查 FAST_LIO_LOCALIZATION2 使用的地图路径：

```bash
cat ~/go2w_nav_ws/scripts/start_fast_lio_localization2.sh
```

应指向：

```bash
/home/luo/go2w_nav_ws/maps/localization/big_localization_raw.pcd
```

检查 jie_3d_nav 地图参数：

```bash
cat ~/go2w_nav_ws/src/jie_3d_nav/octo_planner/config/nav_params.yaml
```

应指向：

```bash
/home/luo/go2w_nav_ws/maps/jie_maps/<导出的地图文件夹>
```

注意：

```text
localization 使用的 PCD 和 jie_3d_nav 使用的地图包，必须来自同一次 PC 端处理流程。
```

---

## 8. 启动完整导航系统

目前工程固定为在pcd地图起点坐标系0点附近启动,若须修改,编辑pub_initialpose_zero.sh
按以下顺序启动。

---

### 8.1 Livox

```bash
chmod +x ~/go2w_nav_ws/scripts/start_livox.sh
~/go2w_nav_ws/scripts/start_livox.sh
```

---

### 8.2 FAST-LIO

```bash
chmod +x ~/go2w_nav_ws/scripts/start_fastlio.sh
~/go2w_nav_ws/scripts/start_fastlio.sh
```

---

### 8.3 FAST_LIO_LOCALIZATION2

```bash
chmod +x ~/go2w_nav_ws/scripts/start_fast_lio_localization2.sh
~/go2w_nav_ws/scripts/start_fast_lio_localization2.sh
```

---

### 8.4 发布 initialpose

```bash
chmod +x ~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
```

---

### 8.5 localization bridge

```bash
chmod +x ~/go2w_nav_ws/scripts/start_localization_bridge.sh
~/go2w_nav_ws/scripts/start_localization_bridge.sh
```

该节点订阅：

```text
/localization
```

并发布：

```text
map -> base_link
```

用于让 jie_3d_nav 和 controller 稳定获取机器人位姿。

注意：

```text
localization bridge 不做地图水平化，也不做角度补偿。
地图水平化已经在 PC 端处理 PCD 时完成。
```

---

### 8.6 jie_3d_nav with controller

```bash
chmod +x ~/go2w_nav_ws/scripts/start_jie_nav_test.sh
~/go2w_nav_ws/scripts/start_jie_nav_test.sh
```

检查 controller：

```bash
ros2 node list | grep d1_controller
ros2 topic info /cmd_vel --verbose
```

---

### 8.7 Go2-W SDK bridge

首先编辑start_go2w_cmd_bridge.sh中network_interface为自己的网络接口名
```bash
chmod +x ~/go2w_nav_ws/scripts/start_go2w_cmd_bridge.sh
~/go2w_nav_ws/scripts/start_go2w_cmd_bridge.sh
```

该节点订阅：

```text
/cmd_vel
/go2w_cmd_enable
```

并在 enable 后调用 Unitree SDK2：

```text
SportClient::Move(vx, vy, vyaw)
```

---

## 9. 运行前检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /localization
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cmd_vel --verbose
ros2 topic info /go2w_cmd_enable --verbose
```

正常情况下应满足：

```text
/livox/lidar 有频率
/livox/imu 有频率
/Odometry 有频率
/localization 有频率
map -> base_link 正常
/cmd_vel 有 d1_controller publisher
/go2w_cmd_enable 有 go2w_cmd_bridge subscriber
```

---

## 10. 使能机器人运动

先用遥控器或官方 App 将 Go2-W 切到爬楼梯模式。

打开网页192.168.123.XX(主机的123网段ip):8080
设置导航目标,开始导航。

若用电脑进入网页会出现d1_controller_xy_tracking可能会卡,点击等待即可。

初次测试建议使用小速度限制，并保证机器人周围空旷。

运行fast-lio过后,操作尽量平稳,若发现已经明显定位漂移,立即停止导航,重新启动.

---

## 11. 常见问题

### 11.1 `./build.sh humble` 权限不够

执行：

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2
chmod +x build.sh
./build.sh humble
```

不要使用：

```bash
sudo ./build.sh humble
```

---

### 11.2 直接 `colcon build --symlink-install` 报 Livox 错误

本项目推荐使用：

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

该脚本会触发当前工作区编译。

不建议把裸的：

```bash
colcon build --symlink-install
```

作为主编译方式。

---

### 11.3 编译时出现很多 `stderr output`

只要最后是：

```text
Summary: xx packages finished
```

且没有：

```text
failed
aborted
```

通常就是成功的。

常见 warning 包括：

```text
DISTRO_ROS / ROS_EDITION unused
rapidjson warning
boost bind deprecated
PCL_ROOT policy warning
```

这些不影响运行。

---

### 11.4 Open3D 找不到

检查：

```bash
python3 -c "import open3d; print(open3d.__version__)"
```

如果失败：

```bash
python3 -m pip install --user open3d --break-system-packages
echo 'export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

---

### 11.5 Go2-W 能 ping 通但不能控制

检查 `enp2s0` IP 顺序：

```bash
ip -br addr show enp2s0
```

推荐：

```text
192.168.123.99/24 192.168.1.50/24
```

同时确认 WiFi 不在：

```text
192.168.123.x
```

网段。

---

### 11.6 有路径但没有 `/cmd_vel`

检查：

```bash
ros2 node list | grep d1_controller
```

如果没有，确认 `start_jie_nav_test.sh` 中启动了 controller，并带有：

```text
launch_controller:=true
```

---

### 11.7 网页端机器人位置不对

检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果没有，启动：

```bash
~/go2w_nav_ws/scripts/start_localization_bridge.sh
```

---

### 11.8 狗不动

检查：

```bash
ros2 node list | grep go2w_cmd_bridge
ros2 topic info /cmd_vel --verbose
ros2 topic info /go2w_cmd_enable --verbose
```

确认已经 enable：

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: true}"
```

如果仍然不动，先单独测试 `/go2w_test_cmd_vel`，确认 SDK bridge 本身能控狗。

---

### 11.9 配置双 IP 时报 `method=manual 不允许 ipv4.addresses 为空`

不要先单独执行：

```bash
sudo nmcli connection modify go2w-mid360 ipv4.method manual
```

因为此时地址为空，NetworkManager 不允许切换到 manual。

请按本文档第 1.3 节的顺序执行：

```bash
sudo nmcli connection add type ethernet ifname enp2s0 con-name go2w-mid360
sudo nmcli connection modify go2w-mid360 ipv4.addresses "192.168.123.99/24,192.168.1.50/24"
sudo nmcli connection modify go2w-mid360 ipv4.method manual
```

先写地址，再改成 manual。

---

### 11.10 出现NumPy版本类报错

保证版本为1.23.5

---


## 12. 快速启动命令汇总

```bash
~/go2w_nav_ws/scripts/start_livox.sh
```

```bash
~/go2w_nav_ws/scripts/start_fastlio.sh
```

```bash
~/go2w_nav_ws/scripts/start_fast_lio_localization2.sh
```

```bash
~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
```

```bash
~/go2w_nav_ws/scripts/start_localization_bridge.sh
```

```bash
~/go2w_nav_ws/scripts/start_jie_nav_test.sh
```

```bash
~/go2w_nav_ws/scripts/start_go2w_cmd_bridge.sh
```

使能：

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: true}"
```

停止：

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: false}"
```

