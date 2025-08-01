# 2D规划服务节点

2D规划服务节点，基于ROS2的nav2_planner包实现路径规划功能。该节点集成了planner_server和costmap配置，提供完整的路径规划服务。

## 功能特性

- 基于nav2_planner的路径规划服务
- 集成全局和局部costmap配置
- 支持多种传感器数据源（激光雷达、点云）
- 可配置的规划算法和参数
- 自动生成临时配置文件
- 智能节点状态管理

## 输入参数

- **scan_topic**: 激光雷达话题名称 (默认: /scan)
- **map_topic**: 地图话题名称 (默认: map)

## 配置参数

### 规划器配置
- **planner_frequency**: 规划频率 (默认: 20.0)
- **planner_plugin**: 规划算法插件 (默认: nav2_navfn_planner::NavfnPlanner)
- **costmap_update_timeout**: costmap更新超时时间 (默认: 1.0)

### 全局costmap配置
- **global_frame**: 全局坐标系 (默认: map)
- **robot_base_frame**: 机器人基座坐标系 (默认: base_link)
- **robot_radius**: 机器人半径 (默认: 0.22)
- **resolution**: 地图分辨率 (默认: 0.05)
- **update_frequency**: 更新频率 (默认: 1.0)
- **publish_frequency**: 发布频率 (默认: 1.0)

### 局部costmap配置
- **local_frame**: 局部坐标系 (默认: odom)
- **rolling_window**: 滚动窗口 (默认: true)
- **width**: 窗口宽度 (默认: 3)
- **height**: 窗口高度 (默认: 3)
- **local_update_frequency**: 局部更新频率 (默认: 5.0)
- **local_publish_frequency**: 局部发布频率 (默认: 2.0)

### 传感器配置
- **obstacle_max_range**: 障碍物最大检测范围 (默认: 2.5)
- **obstacle_min_range**: 障碍物最小检测范围 (默认: 0.0)
- **raytrace_max_range**: 射线追踪最大范围 (默认: 3.0)
- **raytrace_min_range**: 射线追踪最小范围 (默认: 0.0)
- **max_obstacle_height**: 最大障碍物高度 (默认: 2.0)
- **min_obstacle_height**: 最小障碍物高度 (默认: 0.0)

### 膨胀层配置
- **inflation_radius**: 膨胀半径 (默认: 0.55)
- **cost_scaling_factor**: 代价缩放因子 (默认: 1.0)

## 输出
- **plan_topic**: 路径规划结果话题 (默认: /plan_navigation)

## 使用方法

1. 配置输入参数（激光雷达、地图、点云话题）
2. 设置规划器和costmap参数
3. 启动节点，系统会自动：
   - 启动planner_server进程
   - 配置生命周期节点
   - 激活规划服务
   - 发布costmap数据

## 依赖

- ROS2 Humble或更高版本
- nav2_planner包
- nav2_costmap_2d包
- nav2_navfn_planner包

## 安装依赖

在Ubuntu系统上安装必要的nav2包：

```bash
# 安装Navigation2相关包
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
``` 