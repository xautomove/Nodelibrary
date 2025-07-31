# AMCL定位节点

AMCL (Adaptive Monte Carlo Localization) 定位节点，基于ROS2的nav2_amcl包实现机器人定位功能。

## 功能特性

- 基于粒子滤波的机器人定位
- 支持激光雷达和地图输入
- 输出机器人姿态信息
- 可配置的定位参数
- 自动生成临时配置文件

## 输入参数

- **scan_topic**: 激光雷达话题名称 (默认: scan)
- **map_topic**: 地图话题名称 (默认: map)
- **base_frame_id**: 机器人基座坐标系 (默认: base_footprint)
- **odom_frame_id**: 里程计坐标系 (默认: odom)
- **global_frame_id**: 全局坐标系 (默认: map)
- **initial_pose**: 初始位姿设置
- **max_particles**: 最大粒子数 (默认: 2000)
- **min_particles**: 最小粒子数 (默认: 500)

## 输出

- **robot_pose**: 机器人当前位姿信息

## 使用方法

1. 配置输入参数
2. 启动节点
3. 节点将自动启动AMCL定位服务
4. 输出机器人位姿信息

## 依赖

- ROS2 Humble或更高版本
- nav2_amcl包
- nav2_common包

## 安装依赖

在Ubuntu系统上安装必要的nav2包：

```bash
# 安装Navigation2相关包
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
``` 