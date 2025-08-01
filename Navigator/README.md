# 导航器节点

导航器节点，基于ROS2的nav2_bt_navigator包实现导航功能。该节点提供完整的导航服务，包括单点导航和多点导航。

## 功能特性

- 基于nav2_bt_navigator的导航服务
- 支持单点导航（NavigateToPose）
- 支持多点导航（NavigateThroughPoses）
- 可配置的导航参数
- 自动生成临时配置文件
- 智能节点状态管理

## 输入参数

- **global_frame**: 全局坐标系 (默认: map)
- **robot_base_frame**: 机器人基座坐标系 (默认: base_link)

## 配置参数

### 基础配置
- **transform_tolerance**: 坐标变换容差 (默认: 0.1)
- **filter_duration**: 过滤器持续时间 (默认: 0.3)
- **always_reload_bt_xml**: 是否总是重新加载行为树 (默认: false)

### 黑板ID配置
- **goal_blackboard_id**: 目标黑板ID (默认: goal)
- **goals_blackboard_id**: 多目标黑板ID (默认: goals)
- **path_blackboard_id**: 路径黑板ID (默认: path)
- **waypoint_statuses_blackboard_id**: 路径点状态黑板ID (默认: waypoint_statuses)

### 导航器配置
- **navigate_to_pose_enable_groot**: 单点导航Groot监控 (默认: false)
- **navigate_to_pose_groot_port**: 单点导航Groot端口 (默认: 1667)
- **navigate_through_poses_enable_groot**: 多点导航Groot监控 (默认: false)
- **navigate_through_poses_groot_port**: 多点导航Groot端口 (默认: 1669)

## 输出

- **navigator_status**: 导航器状态信息

## 使用方法

1. 配置输入参数（坐标系）
2. 设置导航器参数
3. 启动节点，系统会自动：
   - 启动bt_navigator进程
   - 配置生命周期节点
   - 激活导航服务
   - 提供导航接口

## 依赖

- ROS2 Humble或更高版本
- nav2_bt_navigator包
- nav2_behavior_tree包

## 安装依赖

在Ubuntu系统上安装必要的nav2包：

```bash
# 安装Navigation2相关包
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
``` 