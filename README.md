# PermitNav for Go2-W

Language: [中文](go2w_nav_ws/README_CN.md) | English

**PermitNav** is a rapidly integrated 3D navigation demo for **Unitree Go2-W / Go2W**, built with **Livox MID360**, **ROS2 Humble**, **FAST-LIO**, **FAST_LIO_LOCALIZATION2**, **jie_3d_nav**, and **Unitree SDK2**.

The name **PermitNav** comes from the original motivation of this project: building a working real-robot navigation demo in a very short time to prove that the system could run and to earn permission to leave for an internship.

This project is mainly intended to achieve a basic 3D navigation pipeline on a real Go2-W robot. It is an engineering integration project rather than a from-scratch navigation framework. This project is only meant to achieve a basic 3D navigation demo. My main focus is locomotion, and I have not worked much on navigation before. If bugs appear, restarting the system is probably the best first solution.

Video：
【【开源】PermitNav，缝合速成的一种全局3d导航】 https://www.bilibili.com/video/BV12Y7j6iEHi/?share_source=copy_web&vd_source=b5c22f93b0d2fd36b1d6203208bd650b

The NUC side is responsible for:

```text
MID360 data acquisition
  ↓
FAST-LIO mapping / real-time odometry
  ↓
FAST_LIO_LOCALIZATION2 global localization
  ↓
localization bridge publishing map -> base_link
  ↓
jie_3d_nav Web planning and controller
  ↓
go2w_cmd_bridge forwarding /cmd_vel to Go2-W
```

Default workspace:

```bash
~/go2w_nav_ws
```

---

## Acknowledgements

This project is not a navigation framework implemented from scratch. It is a real-robot ROS2 navigation integration project built quickly on top of existing open-source projects.

We sincerely thank the authors and maintainers of the following projects:

| Open-source project | Usage in this project |
|---|---|
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | Low-level Livox LiDAR SDK used to receive data from MID360 and other Livox LiDARs. |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | ROS/ROS2 driver for Livox LiDARs. In this project, it publishes MID360 point cloud and IMU data. |
| [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | ROS2 version of FAST-LIO. Used for LiDAR-inertial odometry, real-time mapping, `/Odometry`, and registered point cloud publishing. |
| [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) | FAST-LIO based localization framework. In this project, it is adapted for relocalization in a pre-built FAST-LIO point cloud map. |
| [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) | ROS2 Humble based 3D navigation system with Web interaction, OctoMap management, path planning, and controller output. Used for Web goal setting, 3D map management, and navigation planning. |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | Official Unitree SDK2. Used by `go2w_cmd_bridge` to convert `/cmd_vel` into executable velocity commands for Go2-W. |
| [OctoMap](https://octomap.github.io/) | 3D occupancy mapping framework used for 3D environment representation and navigation planning. |

This repository mainly provides:

```text
1. Hardware deployment and network configuration for Go2-W, MID360, and NUC;
2. System integration of Livox, FAST-LIO, FAST_LIO_LOCALIZATION2, jie_3d_nav, and Unitree SDK2;
3. FAST-LIO point cloud map saving, processing, and importing;
4. TF bridging from localization results to the navigation frame;
5. Forwarding /cmd_vel to the Go2-W SDK control interface;
6. A reproducible set of launch scripts and real-robot running procedures.
```

The core value of this project is short-cycle engineering integration, not proposing a new navigation algorithm. It connects multiple existing ROS2 open-source projects into a reproducible real-robot navigation pipeline for Go2-W + MID360.

The original algorithms, drivers, and core software capabilities belong to their respective upstream projects. Please read and comply with the corresponding open-source licenses before use, modification, redistribution, or commercial use.

---

## 1. Hardware and Network

### 1.1 Verified Hardware

| Device | Model |
|---|---|
| Robot | Unitree Go2-W / Go2W |
| LiDAR | Livox MID360 |
| Main computer | N150 NUC / Ubuntu 22.04 |
| ROS | ROS2 Humble |

---

### 1.2 Network Connection

A network switch is recommended:

```text
NUC enp2s0  →  Switch
MID360      →  Switch
Go2-W       →  Switch
```

Current IP configuration:

| Device | IP |
|---|---|
| NUC - Go2-W subnet | `192.168.123.99/24` |
| NUC - MID360 subnet | `192.168.1.50/24` |
| Go2-W | `192.168.123.161` |
| MID360 | `192.168.1.165` |

---

### 1.3 Configure Dual IPs on NUC

This project uses `enp2s0` to connect to both the Go2-W subnet and the MID360 subnet.

It is recommended to create a dedicated wired connection named:

```text
go2w-mid360
```

You can create it from Ubuntu Settings:

```text
Settings -> Network -> Wired -> Add Profile
```

Set the profile name to:

```text
go2w-mid360
```

Then configure it with `nmcli`:

```bash
sudo nmcli connection modify go2w-mid360 ipv4.addresses "192.168.123.99/24,192.168.1.50/24"
sudo nmcli connection modify go2w-mid360 ipv4.method manual
sudo nmcli connection modify go2w-mid360 ipv4.gateway ""
sudo nmcli connection modify go2w-mid360 ipv4.dns ""
sudo nmcli connection modify go2w-mid360 ipv4.never-default yes
sudo nmcli connection modify go2w-mid360 connection.autoconnect yes
sudo nmcli connection up go2w-mid360
```

Check the result:

```bash
ip -br addr show enp2s0
ip route get 192.168.123.161
ip route get 192.168.1.165
ping -c 3 192.168.123.161
ping -c 3 192.168.1.165
```

Expected result:

```text
enp2s0 should have both:
192.168.123.99/24
192.168.1.50/24

Go2-W route source address:
192.168.123.99

MID360 route source address:
192.168.1.50
```

Notes:

```text
1. The Go2-W subnet IP 192.168.123.99/24 should preferably appear first on enp2s0;
2. WiFi should not use the 192.168.123.x subnet;
3. If Go2-W can be pinged but SDK control fails, first check the enp2s0 IP order and possible WiFi subnet conflict.
```

---

## 2. Install Dependencies

### 2.1 Basic Dependencies

```bash
sudo apt update

sudo apt install -y \
  git cmake make gcc g++ build-essential \
  python3-pip python3-setuptools python3-colcon-common-extensions \
  python3-rosdep python3-vcstool python3-argcomplete \
  wget curl unzip tmux gdb net-tools iproute2 pkg-config
```

If apt is locked:

```bash
sudo systemctl stop unattended-upgrades || true
sudo systemctl stop apt-daily.service || true
sudo systemctl stop apt-daily-upgrade.service || true
sudo dpkg --configure -a
sudo apt update
```

---

### 2.2 ROS2 Dependencies

Make sure ROS2 Humble is already installed, then run:

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

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

If rosdep has already been initialized, run:

```bash
rosdep update
```

---

### 2.3 C++ Library Dependencies

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

### 2.4 Python Dependencies

FAST_LIO_LOCALIZATION2 and some map tools require Python dependencies, especially Open3D.

```bash
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-yaml
```

Install Open3D:

```bash
python3 -m pip install --user open3d
```

If you encounter NumPy version errors:

```bash
python3 -m pip uninstall -y numpy scipy scikit-learn sklearn open3d

python3 -m pip install --user --force-reinstall \
  "numpy==1.23.5" \
  "scipy==1.10.1" \
  "scikit-learn==1.3.2" \
  "open3d==0.19.0"
```

Do not upgrade NumPy to 2.x. On Ubuntu 22.04, the combination of SciPy / scikit-learn / Open3D can easily run into ABI conflicts when NumPy 2.x is installed.

Check:

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

If you see:

```text
externally-managed-environment
```

run:

```bash
python3 -m pip install --user open3d --break-system-packages
```

If Open3D still cannot be found after installation:

```bash
echo 'export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

Check again:

```bash
python3 -c "import open3d; print(open3d.__version__)"
```

This step must pass.

Install dependencies for `jie_3d_nav`:

```bash
cd ~/go2w_nav_ws/src/jie_3d_nav
bash install_deps_humble.sh
```

---

## 3. Get the Code

Recommended workspace structure:

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

Enter the workspace:

```bash
cd ~/go2w_nav_ws
```

---

## 4. Build the Project

### 4.1 Install Livox-SDK2

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

---

### 4.2 Install Unitree SDK2

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

---

### 4.3 Build ROS2 Packages

In this project, `livox_ros_driver2` is built using its official build script.

In testing, `./build.sh humble` triggers colcon build in the current workspace and builds the other packages as well. Therefore, this project uses it as the main build method.

Run:

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2

chmod +x build.sh

source /opt/ros/humble/setup.bash

./build.sh humble
```

After the build finishes:

```bash
cd ~/go2w_nav_ws
source install/setup.bash
```

Check the build result:

```bash
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 pkg list | grep fast_lio
ros2 pkg list | grep localization
ros2 pkg executables go2w_cmd_bridge
```

Expected output should include:

```text
livox_ros_driver2/msg/CustomMsg can be displayed
fast_lio
fast_lio_localization
go2w_cmd_bridge go2w_cmd_bridge
```

If `./build.sh humble` ends with:

```text
Summary: xx packages finished
```

and does not contain:

```text
failed
aborted
```

then the build is successful.

`stderr output` during compilation does not necessarily mean failure. Many of them are only CMake warnings.

---

## 5. MID360 Configuration Check

Current MID360 IP:

```text
192.168.1.165
```

NUC host IP:

```text
192.168.1.50
```

Check Livox configuration:

```bash
grep -R "192.168.1" -n ~/go2w_nav_ws/src/livox_ros_driver2/config
```

The configuration should match:

```text
host_ip / cmd_data_ip / point_data_ip / imu_data_ip: 192.168.1.50
lidar_ip: 192.168.1.165
```

Check the network first:

```bash
ip route get 192.168.1.165
ping -c 3 192.168.1.165
```

---

## 6. Mapping Procedure

Before localization and navigation, a FAST-LIO map must be built.

---

### 6.1 Use the FAST-LIO Launch Script with PCD Saving Enabled

The script for saving PCD maps is:

```bash
~/go2w_nav_ws/scripts/start_fastlio.sh.pcd_true
```

Before mapping, copy or rename it as the active FAST-LIO script:

```bash
cp ~/go2w_nav_ws/scripts/start_fastlio.sh.pcd_true \
   ~/go2w_nav_ws/scripts/start_fastlio.sh

chmod +x ~/go2w_nav_ws/scripts/start_fastlio.sh
```

Enable PCD saving in FAST-LIO:

```bash
sed -i 's/pcd_save_en:[[:space:]]*false/pcd_save_en: true/g' \
~/go2w_nav_ws/src/FAST_LIO_ROS2/config/mid360_nuc_mapping.yaml
```

If the installed configuration already exists, modify it as well:

```bash
if [ -f ~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml ]; then
  sed -i 's/pcd_save_en:[[:space:]]*false/pcd_save_en: true/g' \
  ~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml
fi
```

To disable PCD saving later, use the original `start_fastlio.sh` and set:

```bash
sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
~/go2w_nav_ws/src/FAST_LIO_ROS2/config/mid360_nuc_mapping.yaml

sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml
```

---

### 6.2 Start Livox and FAST-LIO for Mapping

Terminal 1:

```bash
chmod +x ~/go2w_nav_ws/scripts/start_livox.sh
~/go2w_nav_ws/scripts/start_livox.sh
```

Terminal 2:

```bash
chmod +x ~/go2w_nav_ws/scripts/start_fastlio.sh
~/go2w_nav_ws/scripts/start_fastlio.sh
```

Check topics:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic list | grep cloud
```

Mapping suggestions:

```text
1. Avoid people walking in the scene during mapping;
2. Move the robot or LiDAR smoothly;
3. Scan corridors, doors, corners, and stairs multiple times;
4. After mapping, shut down FAST-LIO normally and wait until the PCD file is saved.
```

The saved map is located under:

```bash
~/go2w_nav_ws/maps/fastlio_yyyymmdd_hhmmss
```

View the map:

```bash
sudo apt install pcl-tools
pcl_viewer ~/go2w_nav_ws/maps/fastlio_yyyymmdd_hhmmss/scans.pcd
```

Copy `scans.pcd` to the PC for map processing.

The PC side is responsible for:

```text
1. Auto-leveling;
2. Noise removal;
3. Converting the map to OctoMap using jie_3d_nav;
4. Exporting the processed localization PCD and jie_3d_nav map package.
```

See the PC-side README for the detailed processing procedure.

---

## 7. Import Processed Maps from PC

The map processing pipeline is maintained in a separate repository:

| [PermitNav-go2w_nav_pc_ws](https://github.com/real-lsy/PermitNav-go2w_nav_pc_ws) | 

The PC-side project is responsible for processing the raw FAST-LIO map generated on the NUC. After mapping, copy the raw `scans.pcd` from the NUC to the PC workspace and process it in `go2w_nav_pc_ws`.

The PC-side workflow includes:

```text
1. Auto-leveling the raw FAST-LIO point cloud map;
2. Removing floating noise and outlier points;
3. Preparing the localization PCD used by FAST_LIO_LOCALIZATION2;
4. Importing the processed PCD into jie_3d_nav;
5. Converting the map into an OctoMap-based navigation map;
6. Exporting the final localization PCD and jie_3d_nav map folder back to the NUC.
```

After PC-side processing, copy the following files back to the NUC:

```text
1. big_localization_raw.pcd
   → used by FAST_LIO_LOCALIZATION2

2. jie_3d_nav exported map folder
   → used by the Web planner and controller
```

For the detailed PC-side map processing procedure, please refer to the README in the `go2w_nav_pc_ws` project.

---

### 7.1 Import the PCD for Localization

The processed map from the PC side should be:

```text
big_localization_raw.pcd
```

Create the target directory on the NUC:

```bash
mkdir -p ~/go2w_nav_ws/maps/localization
```

Target path:

```bash
~/go2w_nav_ws/maps/localization/big_localization_raw.pcd
```

Run this on the PC:

```bash
scp ~/go2w_nav_pc_ws/maps/final/big_localization_raw.pcd \
  name@<IP>:~/go2w_nav_ws/maps/localization/
```

---

### 7.2 Import the jie_3d_nav Map Folder

The OctoMap map folder exported by jie_3d_nav on the PC side should be placed under:

```bash
~/go2w_nav_ws/maps/jie_maps
```

Create the directory on the NUC:

```bash
mkdir -p ~/go2w_nav_ws/maps/jie_maps
```

Run this on the PC:

```bash
scp -r <PC-side jie_3d_nav exported map folder> \
  name@<IP>:~/go2w_nav_ws/maps/jie_maps/
```

Check:

```bash
ls -lh ~/go2w_nav_ws/maps/localization/
ls -lh ~/go2w_nav_ws/maps/jie_maps/
```

---

### 7.3 Check Map Paths

Check the map path used by FAST_LIO_LOCALIZATION2:

```bash
cat ~/go2w_nav_ws/scripts/start_fast_lio_localization2.sh
```

It should point to:

```bash
～/go2w_nav_ws/maps/localization/big_localization_raw.pcd
```

Check the map path used by jie_3d_nav:

```bash
cat ~/go2w_nav_ws/src/jie_3d_nav/octo_planner/config/nav_params.yaml
```

It should point to:

```bash
～/go2w_nav_ws/maps/jie_maps/<exported_map_folder>
```

Note:

```text
The PCD used by localization and the map package used by jie_3d_nav must come from the same PC-side map processing pipeline.
```

---

## 8. Start the Full Navigation System

The current project assumes that the robot starts near the origin of the PCD map coordinate frame.

If this needs to be changed, edit:

```bash
~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
```

Start the system in the following order.

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

### 8.4 Publish Initial Pose

```bash
chmod +x ~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
~/go2w_nav_ws/scripts/pub_initialpose_zero.sh
```

---

### 8.5 Localization Bridge

```bash
chmod +x ~/go2w_nav_ws/scripts/start_localization_bridge.sh
~/go2w_nav_ws/scripts/start_localization_bridge.sh
```

This node subscribes to:

```text
/localization
```

and publishes:

```text
map -> base_link
```

It allows jie_3d_nav and the controller to obtain the robot pose more reliably.

Note:

```text
The localization bridge does not perform map leveling or angle compensation.
Map leveling should already be completed during PC-side PCD processing.
```

---

### 8.6 jie_3d_nav with Controller

```bash
chmod +x ~/go2w_nav_ws/scripts/start_jie_nav_test.sh
~/go2w_nav_ws/scripts/start_jie_nav_test.sh
```

Check the controller:

```bash
ros2 node list | grep d1_controller
ros2 topic info /cmd_vel --verbose
```

---

### 8.7 Go2-W SDK Bridge

First, edit `start_go2w_cmd_bridge.sh` and set `network_interface` to your actual network interface name.

Then run:

```bash
chmod +x ~/go2w_nav_ws/scripts/start_go2w_cmd_bridge.sh
~/go2w_nav_ws/scripts/start_go2w_cmd_bridge.sh
```

This node subscribes to:

```text
/cmd_vel
/go2w_cmd_enable
```

After being enabled, it calls Unitree SDK2:

```text
SportClient::Move(vx, vy, vyaw)
```

---

## 9. Pre-run Checks

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /localization
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cmd_vel --verbose
ros2 topic info /go2w_cmd_enable --verbose
```

Expected status:

```text
/livox/lidar has frequency
/livox/imu has frequency
/Odometry has frequency
/localization has frequency
map -> base_link is available
/cmd_vel has a d1_controller publisher
/go2w_cmd_enable has a go2w_cmd_bridge subscriber
```

---

## 10. Enable Robot Motion

First, use the remote controller or the official Unitree App to switch Go2-W to stair-climbing mode.

Open the Web interface:

```text
http://192.168.123.XX:8080
```

where `192.168.123.XX` is the host IP in the `192.168.123.x` subnet.

Set the navigation goal and start navigation.

If the Web interface freezes at `d1_controller_xy_tracking`, click wait.

For the first test, use a small velocity limit and make sure the surrounding area is clear.

After FAST-LIO starts running, operate the robot smoothly. If obvious localization drift is observed, stop navigation immediately and restart the localization/navigation pipeline.

---

## 11. Troubleshooting

### 11.1 `./build.sh humble` Permission Denied

Run:

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2
chmod +x build.sh
./build.sh humble
```

Do not use:

```bash
sudo ./build.sh humble
```

---

### 11.2 Livox Error When Running Plain `colcon build --symlink-install`

This project recommends:

```bash
cd ~/go2w_nav_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

This script triggers the build of the current workspace.

It is not recommended to use plain:

```bash
colcon build --symlink-install
```

as the main build method.

---

### 11.3 Many `stderr output` Messages During Build

If the final output is:

```text
Summary: xx packages finished
```

and does not contain:

```text
failed
aborted
```

then the build is usually successful.

Common warnings include:

```text
DISTRO_ROS / ROS_EDITION unused
rapidjson warning
boost bind deprecated
PCL_ROOT policy warning
```

These warnings do not affect normal running.

---

### 11.4 Open3D Cannot Be Found

Check:

```bash
python3 -c "import open3d; print(open3d.__version__)"
```

If it fails:

```bash
python3 -m pip install --user open3d --break-system-packages
echo 'export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

---

### 11.5 Go2-W Can Be Pinged but Cannot Be Controlled

Check the IP order on `enp2s0`:

```bash
ip -br addr show enp2s0
```

Recommended order:

```text
192.168.123.99/24 192.168.1.50/24
```

Also make sure WiFi is not using:

```text
192.168.123.x
```

---

### 11.6 A Path Exists but `/cmd_vel` Is Missing

Check:

```bash
ros2 node list | grep d1_controller
```

If it is not running, make sure `start_jie_nav_test.sh` starts the controller with:

```text
launch_controller:=true
```

---

### 11.7 Robot Position Is Incorrect in the Web Interface

Check:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

If there is no transform, start:

```bash
~/go2w_nav_ws/scripts/start_localization_bridge.sh
```

---

### 11.8 The Robot Does Not Move

Check:

```bash
ros2 node list | grep go2w_cmd_bridge
ros2 topic info /cmd_vel --verbose
ros2 topic info /go2w_cmd_enable --verbose
```

Make sure motion is enabled:

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: true}"
```

If it still does not move, test `/go2w_test_cmd_vel` separately to confirm that the SDK bridge can control the robot.

---

### 11.9 Error When Configuring Dual IPs: `method=manual is not allowed when ipv4.addresses is empty`

Do not run this first:

```bash
sudo nmcli connection modify go2w-mid360 ipv4.method manual
```

At that moment, the address field is empty, and NetworkManager does not allow switching to manual mode.

Follow the order in Section 1.3:

```bash
sudo nmcli connection add type ethernet ifname enp2s0 con-name go2w-mid360
sudo nmcli connection modify go2w-mid360 ipv4.addresses "192.168.123.99/24,192.168.1.50/24"
sudo nmcli connection modify go2w-mid360 ipv4.method manual
```

Set the addresses first, then set the method to manual.

---

### 11.10 NumPy Version Error

Make sure NumPy is:

```text
1.23.5
```

---

## 12. Quick Start Commands

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

Enable motion:

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: true}"
```

Stop motion:

```bash
ros2 topic pub --once /go2w_cmd_enable std_msgs/msg/Bool "{data: false}"
```

## License

This repository is released under the Apache License, Version 2.0.

The Apache License 2.0 applies only to the original integration code, scripts, configuration files, bridge nodes, and documentation created for PermitNav.

Third-party source code included in this repository, such as FAST-LIO, FAST_LIO_LOCALIZATION, FAST_LIO_ROS2, livox_ros_driver2, Livox-SDK2, jie_3d_nav, unitree_sdk2, and OctoMap-related components, remains governed by its original upstream license.

Users must check and comply with the corresponding upstream licenses before redistribution, modification, or commercial use.
