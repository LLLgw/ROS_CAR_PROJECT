# ROS_CAR_PROJECT

这是一个面向移动机器人、边缘计算与低空智能巡航监测场景的综合工程仓库。仓库整合了 ROS 2 机器人端软件、RK3588/RK3576 边缘计算与视觉推理代码、STM32F4 底盘控制固件、硬件设计资料以及项目技术文档。

项目整体围绕智能小车/低空巡航监测平台构建，覆盖底盘控制、传感器接入、SLAM 建图、导航、激光雷达点云处理、YOLO/RKNN 目标检测、云台控制和软硬件系统集成等内容。

## 仓库结构

```text
ROS_CAR_PROJECT/
|-- doc/                 项目技术文档
|-- firmware/            STM32F4 + FreeRTOS 底盘控制固件
|-- hardware/            硬件原理图、PCB 与电源板相关资料
|-- robot_ws/            ROS 2 与边缘计算相关工作区
|   `-- src/
|       |-- RK3588/      RK3588 机器人端 ROS 2 功能包
|       `-- rk3576/      RK3576 视觉推理、云台与 Web 展示相关代码
`-- README.md            仓库总说明
```

## 主要内容

### `robot_ws/src/RK3588`

该目录是 RK3588 机器人端的 ROS 2 软件部分，主要包含底盘通信、传感器启动、激光雷达接入、SLAM 建图、Navigation2 导航、点云地面分割以及 YOLOv11 RKNN 目标检测等模块。

包含的主要功能包有：

- `robot_low_connect`：机器人底层连接、URDF、硬件参数和传感器启动配置
- `robot_cartographer`：Cartographer 建图与 EKF 融合配置
- `robot_navigation2`：Navigation2 导航参数、地图和启动配置
- `rslidar_sdk` / `rslidar_msg`：RS 激光雷达驱动与消息定义
- `yolov11_rknn_ros2`：基于 RKNN 的 YOLOv11 ROS 2 检测节点
- `patchwork-plusplus`：点云地面分割相关源码与 ROS 2 集成

### `robot_ws/src/rk3576`

该目录主要面向 RK3576 平台的视觉推理、摄像头 Web 服务、无人机目标检测和云台控制实验。

其中包含多个相对独立的子工作区和工具目录，例如：

- `rknn_yolo11`：RKNN YOLOv11 推理相关代码
- `camera_web_ws` / `camera_web_cpp_ws`：摄像头图像 Web 发布相关工程
- `yolo_web_py_ws` / `yolo_web_cpp_ws`：Python/C++ 版本 YOLO Web 推理工程
- `yolo_web_py_canvas_ws`：带 Canvas 显示的 YOLO Web 推理实现
- `drone_yolo_web_cpp_ws`：无人机检测方向的 YOLO Web C++ 工程
- `dm_h3510_ros_ws` / `gimbal_dm_h3510_ws`：DM-H3510 云台控制与 ROS 集成
- `drone_pt_detector`：无人机目标检测相关模块
- `docs`、`scripts`、`tools`：文档、脚本和辅助工具

### `firmware`

`firmware` 是小车底盘控制固件，基于 STM32F4 与 FreeRTOS 实现。该部分负责底层运动控制、ROS 串口通信、PS2 手柄输入、电池检测、OLED 显示和急停保护等功能。

主要目录包括：

- `USER`：主函数、全局状态、FreeRTOS 配置和中断处理
- `BSP`：电机、编码器、串口、USB、OLED、急停等底层驱动
- `DRIVE`：底盘运动控制与 PI 调速
- `ROS`：ROS 串口协议解析与状态上报
- `INPUT`：PS2 手柄输入处理
- `BATTERY`：电池 ADC 采样与状态更新
- `DISPLAY`：OLED 显示任务
- `FreeRTOS`：FreeRTOS 内核源码
- `FWLIB`、`CORE`、`SYSTEM`：STM32 标准外设库、CMSIS 与系统基础模块

### `hardware`

硬件资料目录，用于存放项目相关的原理图、PCB 和电源板资料。当前包含硬件原理图 PDF、PCB 设计 PDF 以及硬件目录说明文件。

### `doc`

项目文档目录，用于存放系统设计、平台架构、功能说明和技术文档等资料。当前包含低空无人机智能巡航监测一体化平台相关技术文档。

## 技术组成

本仓库涉及的主要技术方向包括：

- ROS 2 机器人软件开发
- RK3588 / RK3576 边缘计算部署
- RKNN 模型推理与 YOLO 目标检测
- Cartographer SLAM 与 Navigation2 导航
- 激光雷达点云处理与地面分割
- STM32F4 嵌入式控制
- FreeRTOS 多任务调度
- 电机闭环控制、编码器反馈与底盘通信
- 云台控制、摄像头采集与 Web 可视化

## 依赖与环境

本仓库包含多个平台和子工程，实际依赖会随使用的模块不同而变化。整体上主要涉及以下环境与依赖：

### ROS 2 与机器人端软件

- Ubuntu 22.04 或其他兼容 ROS 2 Humble 的 Linux 环境
- ROS 2 Humble
- `colcon` 构建工具
- CMake、GCC/G++、Python 3
- `rclcpp`、`rclpy`、`launch`、`launch_ros` 等 ROS 2 基础组件
- `tf2`、`robot_state_publisher`、`joint_state_publisher`、`rviz2`
- `sensor_msgs`、`geometry_msgs`、`nav_msgs`、`std_msgs` 等常用消息包
- `robot_localization`、Cartographer、Navigation2 等建图与导航相关组件

### 视觉推理与边缘计算

- RK3588 / RK3576 Linux 系统环境
- Rockchip NPU 驱动与 RKNN Runtime
- RKNN Toolkit / RKNN Toolkit Lite2
- OpenCV
- NumPy 等 Python 推理与图像处理依赖
- YOLOv11 / RKNN 模型文件
- 摄像头、V4L2 或平台对应的视频采集接口

### 激光雷达与点云处理

- RS 激光雷达硬件及其网络/串口连接环境
- `rslidar_sdk` 及对应 ROS 2 消息包
- PCL 点云库
- Eigen
- yaml-cpp
- RViz / RViz2 可视化环境
- Patchwork++ 点云地面分割相关依赖

### 硬件设备

- RK3588 / RK3576 边缘计算板卡
- 电机、编码器、电机驱动、电池与急停开关
- 激光雷达、IMU、GPS、摄像头等传感器
- DM-H3510 云台或项目中使用的对应云台设备

