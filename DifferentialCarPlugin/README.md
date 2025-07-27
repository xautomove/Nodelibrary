# 差速驱动小车插件 (DifferentialCarPlugin)

一个基于Gazebo仿真环境的差速驱动小车模型，采用前万向轮 + 后差速驱动的设计。

## 功能特性

- **差速驱动系统**: 通过后轮差速控制车辆前进、后退和转向
- **万向轮支撑**: 前置万向轮提供稳定支撑，无阻力设计
- **激光雷达**: 360度扫描，用于环境感知和导航
- **IMU传感器**: 提供姿态和运动信息
- **键盘控制**: 支持WASD键盘控制
- **ROS2集成**: 完整的ROS2话题通信支持
- **YAML配置**: 动态传感器参数配置

## 依赖安装

### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo 11
- Python 3.10+

### 安装依赖
```bash
# 安装Gazebo相关包
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-plugins

# 安装XACRO工具
sudo apt install ros-humble-xacro

# 安装URDF工具
sudo apt install liburdfdom-tools

# 安装Python依赖
pip install pygame pyyaml
```

## 文件结构

```
DifferentialCarPlugin/
├── config/
│   ├── imu.yaml          # IMU传感器配置
│   └── lidar.yaml        # 激光雷达配置
├── urdf/
│   ├── differential_car.urdf.xacro  # 车辆XACRO模型文件
│   └── differential_car.urdf        # 自动生成的URDF文件
├── worlds/
│   └── empty.world       # Gazebo仿真世界
├── config.json          # 插件基本配置
├── main.py              # 主程序
└── README.md           # 说明文档
```

## 控制方式

### 键盘控制
- **W**: 前进
- **S**: 后退  
- **A**: 左转
- **D**: 右转
- **ESC**: 退出

## 使用方法

### 启动仿真
```bash
cd DifferentialCarPlugin
python3 main.py
```

### 验证URDF文件
```bash
check_urdf urdf/differential_car.urdf
```

## 配置文件

### 激光雷达配置 (config/lidar.yaml)
```yaml
lidar:
  topic: "/scan"
  frame_id: "lidar_link"
  update_rate: 10
  min_angle: -3.14159
  max_angle: 3.14159
  samples: 360
```

### IMU配置 (config/imu.yaml)
```yaml
imu:
  topic: "/imu"
  frame_id: "imu_link"
  update_rate: 50
```

## 技术规格

### 车体参数
- 车身尺寸: 0.5m × 0.3m × 0.1m
- 后轮半径: 0.05m
- 万向轮半径: 0.05m
- 轮距: 0.3m
- 轴距: 0.3m

### 传感器配置
- **激光雷达**: 360度扫描，10Hz更新频率，10m检测范围
- **IMU**: 50Hz更新频率，提供姿态和加速度信息

### ROS2话题
- `/cmd_vel`: 速度控制命令 (geometry_msgs/Twist)
- `/scan`: 激光雷达数据 (sensor_msgs/LaserScan)
- `/imu`: IMU数据 (sensor_msgs/Imu)
- `/odom`: 里程计数据 (nav_msgs/Odometry)

## 故障排除

### 常见问题

1. **传感器话题没有数据**
   - 检查Gazebo插件路径设置
   - 确认ROS2环境变量正确

2. **XACRO转换失败**
   ```bash
   sudo apt install ros-humble-xacro
   ```

3. **激光雷达闪烁**
   - 调整世界文件中的物理引擎步长
   - 检查传感器更新频率配置

4. **pygame模块缺失**
   ```bash
   pip install pygame
   ```

5. **ROS2环境问题**
   ```bash
   source /opt/ros/humble/setup.bash
   export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
   ```