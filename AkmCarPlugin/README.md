# AkmCarPlugin - 自动驾驶仿真测试插件

## 概述

AkmCarPlugin是一个基于Gazebo和ROS2的自动驾驶仿真测试插件，专为**FutureDrive**平台设计，旨在为学习者提供一个完整的自动驾驶车辆仿真环境。通过该插件，用户可以深入了解激光雷达、IMU传感器的工作原理，以及差分驱动车辆的运动控制机制。

> **重要提示**：此插件需要在FutureDrive环境中运行，请确保已正确安装和配置FutureDrive平台。

## 主要功能

### 🚗 车辆模型
- **差分驱动系统**：采用后轮驱动，前轮自由转向的设计
- **物理仿真**：真实的车轮摩擦力、惯性和动力学特性
- **视觉区分**：前轮（红色）用于转向，后轮（黑色）用于驱动

### 📡 传感器集成
- **激光雷达**：360度扫描，检测障碍物和环境信息
  - 扫描范围：0.12m - 6.0m
  - 扫描频率：可配置（默认10Hz）
  - 360个采样点，1度分辨率
- **IMU传感器**：提供车辆姿态和运动信息
  - 角速度和线性加速度测量
  - 高斯噪声模拟真实传感器特性
  - 更新频率：可配置（默认50Hz）

### 🌍 测试环境
- **丰富的障碍物场景**：包含围墙、方形障碍物、圆柱形障碍物
- **封闭测试区域**：12x10米的测试空间
- **多样化地形**：为SLAM建图和路径规划提供理想测试环境

## 文件结构

```
AkmCarPlugin/
├── config.json          # 插件基本信息
├── main.py              # 主启动脚本
├── README.md            # 本文档
├── config/              # 传感器配置目录
│   ├── imu.yaml         # IMU传感器配置
│   └── lidar.yaml       # 激光雷达配置
├── urdf/                # 车辆模型定义
│   └── akm_car.urdf     # 车辆URDF模型文件
└── worlds/              # Gazebo世界文件
    └── empty.world      # 带障碍物的测试环境
```

## 系统要求

### FutureDrive平台
此插件专为**FutureDrive**自动驾驶开发平台设计，需要：
- FutureDrive平台已正确安装
- FutureDrive插件管理系统
- 支持ROS2 Humble的Linux环境

### 依赖安装

```bash
# 安装ROS2 Gazebo插件
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# 安装Python依赖
pip install PyYAML
```

> **注意**：如果在FutureDrive平台中运行，部分依赖可能已预装。

## 使用方法

### 1. 在FutureDrive中使用

#### 方式一：通过FutureDrive插件安装器
1. 在FutureDrive平台中找到插件扩展安装页面
2. 在本页右上角复制命令
3. 在客户端点击安装
3. 在客户端选择Gazebo仿真，使用main.py作为启动文件

#### 方式二：手动启动
```bash
cd AkmCarPlugin
python3 main.py
```

该命令会：
- 读取yaml配置文件
- 启动Gazebo仿真环境
- 自动加载车辆模型和传感器
- 与FutureDrive平台进行数据交互

### 2. 车辆控制

通过ROS2话题控制车辆运动：

```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}'

# 后退
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: -1.0}}'

# 原地左转
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'

# 原地右转
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: -0.5}}'

# 前进并左转
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'
```

### 3. 传感器数据监控

查看激光雷达数据：
```bash
ros2 topic echo /scan
```

查看IMU数据：
```bash
ros2 topic echo /imu
```

查看里程计信息：
```bash
ros2 topic echo /odom
```

## 配置自定义

### 修改传感器参数

编辑 `config/lidar.yaml` 调整激光雷达：
```yaml
lidar:
  topic: "/scan"
  frame_id: "lidar_link"
  update_rate: 10
  min_angle: -3.14
  max_angle: 3.14
  samples: 360
```

编辑 `config/imu.yaml` 调整IMU：
```yaml
imu:
  topic: "/imu"
  frame_id: "imu_link"
  update_rate: 50
```

### 修改世界环境

编辑 `worlds/empty.world` 添加或修改障碍物，创建不同的测试场景。

## 学习目标

通过使用此插件，用户可以学习到：

1. **传感器融合**：理解激光雷达和IMU在自动驾驶中的作用
2. **SLAM算法**：使用激光雷达数据进行同时定位与建图
3. **路径规划**：在已知地图中规划避障路径
4. **车辆动力学**：理解差分驱动的运动原理
5. **ROS2生态**：掌握ROS2的话题通信和仿真工具链

## 扩展应用

### 在FutureDrive平台中的应用
- **自动导航**：集成ROS2 Navigation Stack实现自主导航
- **SLAM建图**：使用slam_toolbox或cartographer进行建图
- **目标跟踪**：添加摄像头传感器实现视觉目标跟踪
- **多车协同**：部署多个车辆实现协同控制
- **强化学习**：作为环境训练自动驾驶AI模型
- **FutureDrive集成**：与FutureDrive的其他模块（路径规划、决策控制等）无缝协作

## 技术特点

- **FutureDrive原生支持**：专为FutureDrive平台优化，支持平台标准接口
- **模块化设计**：传感器配置与URDF模型分离，便于修改
- **真实物理仿真**：精确的摩擦力、惯性和碰撞检测
- **易于扩展**：可轻松添加新传感器或修改车辆参数
- **标准ROS2接口**：兼容现有的ROS2自动驾驶软件栈
- **平台集成**：与FutureDrive的数据流和控制流无缝集成

## 故障排除

### 常见问题

1. **插件无法在FutureDrive中启动**
   - 检查FutureDrive平台是否正确安装
   - 确认插件权限和依赖是否满足
   - 查看FutureDrive日志获取详细错误信息

2. **车辆不动**
   - 检查ROS2环境是否正确加载
   - 确认Gazebo插件是否正确安装
   - 验证FutureDrive与插件的通信状态

3. **传感器数据异常**
   - 检查yaml配置文件格式
   - 确认传感器插件库是否存在
   - 验证FutureDrive数据接口配置

4. **车辆运动不稳定**
   - 调整轮子摩擦力参数
   - 检查车辆质量和惯性设置
   - 确认FutureDrive控制参数配置

### 调试命令

```bash
# 查看所有可用话题
ros2 topic list

# 查看节点信息
ros2 node list

# 监控TF变换
ros2 run tf2_tools view_frames
```

## FutureDrive集成说明

此插件作为FutureDrive生态系统的一部分，提供：
- 标准化的传感器数据接口
- 与FutureDrive控制模块的无缝集成
- 支持FutureDrive的插件生命周期管理
- 兼容FutureDrive的数据记录和回放功能

更多FutureDrive平台相关信息，请参考FutureDrive官方文档。

## 贡献

欢迎提交问题报告和功能改进建议。该插件是为教育目的设计，适合自动驾驶初学者在FutureDrive平台中使用。

## 许可证

本项目采用开源许可证，供学习和研究使用。使用时请遵循FutureDrive平台的相关协议。 