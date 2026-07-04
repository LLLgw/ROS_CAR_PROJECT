# 源码工作区说明

该目录用于存放小车项目的 ROS 2 源码包，是 `robot_ws` 工作区中的核心源码目录。

## 目录结构

- `robot_cartographer`：Cartographer 建图与 SLAM 相关的启动文件、参数配置和地图处理内容。
- `robot_low_connect`：机器人底层连接、硬件参数、URDF 模型以及设备启动相关内容。
- `robot_navigation2`：Navigation2 导航配置、启动文件与地图资源。
- `rslidar_msg`：RS 激光雷达相关的自定义消息定义。
- `rslidar_sdk`：RS 激光雷达 SDK 在 ROS 2 中的集成包。
- `yolov11_rknn_ros2`：基于 RKNN 的 YOLOv11 ROS 2 目标检测节点。
- `patchwork-plusplus`：Patchwork++ 点云地面分割相关源码。

## 编译与环境准备

在启动任意功能包前，建议先完成工作区编译并加载环境：

```bash
cd robot_ws
colcon build
source install/setup.bash
```

如果已经编译过，通常至少需要重新执行：

```bash
cd robot_ws
source install/setup.bash
```

## 各包启动方式

### 1. `robot_low_connect`

用于启动机器人底层通信、IMU、GPS、雷达点云转二维激光以及 RViz。

完整启动：

```bash
ros2 launch robot_low_connect robot_start_launch.py
```

仅启动雷达、点云转激光和 RViz：

```bash
ros2 launch robot_low_connect rs_start.py
```


### 2. `robot_cartographer`

用于建图与定位过程中的 Cartographer 启动。

```bash
ros2 launch robot_cartographer cartographer_launch.py
```

该启动文件会同时拉起：

- `cartographer_node`
- `cartographer_occupancy_grid_node`
- `robot_localization` 中的 EKF 节点

### 3. `robot_navigation2`

用于导航功能启动，默认会读取地图与 Nav2 参数文件。

```bash
ros2 launch robot_navigation2 navigation2_launch.py
```

该启动文件默认使用：

- 地图：`robot_navigation2/maps/my_map.yaml`
- 参数：`robot_navigation2/param/robot_nav2.yaml`
- EKF 配置：`robot_navigation2/config/ekf_nav.yaml`

### 4. `yolov11_rknn_ros2`

用于启动 RKNN 版 YOLOv11 目标检测节点。

```bash
ros2 launch yolov11_rknn_ros2 yolov11_rknn.launch.py
```

### 5. `patchwork-plusplus`

用于点云地面分割。

```bash
ros2 launch patchworkpp patchworkpp.launch.py
```

## 推荐启动顺序

根据常见使用场景，可以参考下面顺序：

### 底盘与传感器启动

```bash
ros2 launch robot_low_connect robot_start_launch.py
```

### 建图模式

先启动底层与雷达，再启动：

```bash
ros2 launch robot_cartographer cartographer_launch.py
```

### 导航模式

先确保底层、雷达、定位和地图配置正常，再启动：

```bash
ros2 launch robot_navigation2 navigation2_launch.py
```

### 视觉检测模式

在相机设备和 RKNN 模型准备完成后启动：

```bash
ros2 launch yolov11_rknn_ros2 yolov11_rknn.launch.py
```

### 点云地面分割模式

在雷达点云话题正常发布后启动：

```bash
ros2 launch patchworkpp patchworkpp.launch.py
```

## 说明

- 本目录需要作为 `robot_ws` 工作区的一部分进行编译。
- 部分功能包主要提供启动文件、配置文件和参数文件，因此这里用该 README 统一说明整体结构和启动方式。